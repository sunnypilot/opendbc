"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car.honda.values import HONDA_ELESYS, STEER_THRESHOLD
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP

# GW_ACTIVE is low priority on the board and has been observed dropping, so the window is
# 500 ms rather than the 300 ms used for SP_HUD_STATUS in the other direction. carstate runs
# at 100 Hz, so that is 50 frames.
LINBUS_GW_STALE_FRAMES = 50

# GW_STEER_GRANT (0x70B) is 10 Hz like GW_ACTIVE and gets the same window, for the same
# reason. SP-PROTOCOL-V3 section 2 rule 8: "Treat a missing 0x70B or 0x704 as not granted,
# with a 500 ms timeout. Absence is never permission."
LINBUS_GRANT_STALE_FRAMES = 50

# GW_STEER_GRANT.STATE. Only these three mean the board is, or is about to be, putting
# torque on the serial line.
GRANT_STATES_STEERING = (3, 4, 5)   # INTRO, ACTIVE, LIMITED

# RETRY_IN: seconds until a new request is considered. This value means "not this key cycle",
# and it is the ONLY thing the board says that means that -- see latchedUntilKeyOff below.
GRANT_RETRY_KEY_CYCLE = 255

# While the EPS is under LKAS control it stops updating STEER_TORQUE_SENSOR on 0x18F: the
# frame keeps arriving at 100 Hz with a rolling counter and a valid checksum, but the torque
# and angle-rate bytes hold the value they had when LKAS engaged. Routes dd/de/df: frozen for
# up to 946 s while the wheel moved -13.5 to +7.4 deg, 41-69 % of each drive, canValid 1.00.
#
# 0.25 s of a bit-identical value is already far outside normal: with the EPS reporting
# properly the value changes at 29-83 Hz and the 95th percentile hold is 1-2 frames.
STEER_TORQUE_STALE_FRAMES = 25

# EPS_LIN_RAW (0x700) is the board's mirror of the EPS serial frame, one CAN frame per serial
# frame, so 100 Hz -- the same rate as the message it stands in for. Half the window of the
# 10 Hz frames for that reason.
EPS_LIN_RAW_STALE_FRAMES = 25

# EPS_LIN_RAW.STEER_TORQUE is in serial counts, scale 2 already applied by the parser, and
# LEFT NEGATIVE. CarState.steeringTorque is in this car's CAN counts and LEFT POSITIVE.
# Least squares over the 30 482 samples of routes dd/de/df where both signals were live:
# 0x18F = -64.52 * STEER_TORQUE, R^2 0.9991, residual RMS 134 CAN counts against a 600-1200
# count threshold. Converting into the CAN domain rather than rescaling every threshold keeps
# STEER_THRESHOLD, torqued, driver monitoring and the lane-change nudge working unchanged.
SERIAL_TORQUE_TO_CAN = -64.5

# SCM_BUTTONS.FUEL_LEVEL is clamped by the meter at 105 (~52 L of a ~60 L tank), so this is
# "fraction of the gauge", not fraction of the tank. See _nidec_scm_group_a_elesys.dbc.
FUEL_LEVEL_FULL = 105.0


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self._linbus_gw_stale = LINBUS_GW_STALE_FRAMES
    self._linbus_gw_ts = 0
    self._linbus_grant_stale = LINBUS_GRANT_STALE_FRAMES
    self._linbus_grant_ts = 0
    self._linbus_grant_logged = None
    self._steer_torque_last = None
    self._steer_torque_held = 0
    self._eps_lin_stale = EPS_LIN_RAW_STALE_FRAMES
    self._eps_lin_ts = 0

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP,
             can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if self.CP.carFingerprint in HONDA_ELESYS:
      self._update_linbus_gateway(ret_sp, cp)
      self._update_linbus_grant(ret_sp, cp)
      self._update_linbus_firmware(ret_sp, cp)
      self._update_driver_torque_validity(ret, ret_sp, cp)
      ret.fuelGauge = min(cp.vl["SCM_BUTTONS"]["FUEL_LEVEL"] / FUEL_LEVEL_FULL, 1.0)

    if self.CP_SP.flags & HondaFlagsSP.NIDEC_HYBRID:
      ret.accFaulted = bool(cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_1"] or cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_2"])
      ret.stockAeb = bool(cp_cam.vl["BRAKE_COMMAND"]["AEB_REQ_1"] and cp_cam.vl["BRAKE_COMMAND"]["COMPUTER_BRAKE_HYBRID"] > 1e-5)

    if self.CP_SP.flags & HondaFlagsSP.HYBRID_ALT_BRAKEHOLD:
      ret.brakeHoldActive = cp.vl["BRAKE_HOLD_HYBRID_ALT"]["BRAKE_HOLD_ACTIVE"] == 1

    if self.CP_SP.enableGasInterceptor:
      # Same threshold as panda, equivalent to 1e-5 with previous DBC scaling
      gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = gas > 492

  def _update_driver_torque_validity(self, ret: structs.CarState, ret_sp: structs.CarStateSP, cp: CANParser) -> None:
    """Refuse to read driver intent out of a latched STEER_TORQUE_SENSOR.

    Gated on STEER_CONTROL_ACTIVE, which is the EPS's own statement that it is under LKAS
    control, AND on the value actually having stopped moving -- either alone would be wrong.
    STEER_CONTROL_ACTIVE alone would discard good data on any car/firmware that keeps
    reporting; a stale value alone would fire when the driver genuinely holds a constant
    torque off-LKAS.

    steeringPressed is forced False rather than True because every consumer treats True as
    "the driver is doing something", and acting on a 15-minute-old sample is worse than
    acting on none: it freezes the lateral integrator for the whole engagement, tells
    driver monitoring the driver is holding the wheel, stops torqued and lagd learning, and
    silences the "openpilot is undershooting, take over" alert in selfdrived.

    This is a mitigation, NOT a fix. With the value latched there is no driver-torque signal
    on CAN at all, so openpilot cannot see the driver fight it either way. The fix is the
    board publishing the driver torque it already decodes off the EPS serial frame --
    docs/SP_GATEWAY_FIRMWARE.md section 6.2.
    """
    torque = cp.vl["STEER_STATUS"]["STEER_TORQUE_SENSOR"]
    if torque != self._steer_torque_last:
      self._steer_torque_last = torque
      self._steer_torque_held = 0
    else:
      self._steer_torque_held = min(self._steer_torque_held + 1, STEER_TORQUE_STALE_FRAMES)

    under_lkas_control = bool(cp.vl["STEER_STATUS"]["STEER_CONTROL_ACTIVE"])
    latched = under_lkas_control and self._steer_torque_held >= STEER_TORQUE_STALE_FRAMES

    if latched and self._eps_lin_driver_torque_valid(cp):
      # Same quantity, live, at the same 100 Hz, from the one device that can still see it.
      ret.steeringTorque = SERIAL_TORQUE_TO_CAN * cp.vl["EPS_LIN_RAW"]["STEER_TORQUE"]
      ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD.get(self.CP.carFingerprint, 1200)
      latched = False

    ret_sp.driverTorqueStale = latched
    if latched:
      ret.steeringPressed = False

  def _eps_lin_driver_torque_valid(self, cp: CANParser) -> bool:
    """Is the board's EPS mirror fresh enough, and did the EPS frame behind it check out?

    Frame-counted like every other gateway frame, because CarState.update() is not handed a
    clock. A board that stops talking must read as no substitute rather than as a torque of
    whatever it last said -- which is the exact failure this whole function exists to undo.
    """
    ts = cp.ts_nanos["EPS_LIN_RAW"]["STEER_TORQUE"]
    if ts != self._eps_lin_ts:
      self._eps_lin_ts = ts
      self._eps_lin_stale = 0
    else:
      self._eps_lin_stale = min(self._eps_lin_stale + 1, EPS_LIN_RAW_STALE_FRAMES)

    fresh = self._eps_lin_stale < EPS_LIN_RAW_STALE_FRAMES and ts != 0
    return fresh and bool(cp.vl["EPS_LIN_RAW"]["CHECKSUM_OK"])

  def _update_linbus_gateway(self, ret_sp: structs.CarStateSP, cp: CANParser) -> None:
    """Decode GW_ACTIVE (0x704) from the aftermarket LIN-bus gateway.

    Staleness is counted in frames rather than against a clock, because CarState.update()
    is not handed a timestamp. ts_nanos only advances when a frame actually arrives, so a
    ts that stops moving is a gateway that stopped talking -- which must read as NOT
    actuating, or openpilot would keep integrating against a car that is no longer
    following it. That is the failure this whole protocol exists to prevent.
    """
    gw = cp.vl["GW_ACTIVE"]          # registered liveness-exempt in get_can_parsers()
    ts = cp.ts_nanos["GW_ACTIVE"]["ENGAGED"]
    if ts != self._linbus_gw_ts:
      self._linbus_gw_ts = ts
      self._linbus_gw_stale = 0
    else:
      self._linbus_gw_stale = min(self._linbus_gw_stale + 1, LINBUS_GW_STALE_FRAMES)

    valid = self._linbus_gw_stale < LINBUS_GW_STALE_FRAMES and ts != 0
    engaged = bool(gw["ENGAGED"])
    dry_run = bool(gw["DRY_RUN"])

    ret_sp.linbusGateway.present = True
    ret_sp.linbusGateway.engaged = engaged
    ret_sp.linbusGateway.dryRun = dry_run
    ret_sp.linbusGateway.valid = valid
    # The one flag consumers read. In a dry run the board still sets ENGAGED -- it reports
    # what it WOULD do -- so ENGAGED alone would tell openpilot it is in control when it is
    # not, and the integrator would wind up exactly as it did before this protocol existed.
    ret_sp.linbusGateway.actuating = engaged and not dry_run and valid

  def _update_linbus_firmware(self, ret_sp: structs.CarStateSP, cp: CANParser) -> None:
    """Decode GW_VERSION (0x707) and GW_BUILD (0x70F) -- which firmware, and which board.

    NO STALENESS WINDOW HERE, deliberately, and it is the one thing that makes this method
    different from the two above. Those decode 10 Hz control frames where a frame that stops
    arriving means the board stopped talking, so silence has to read as "not granted". These
    are 1/min identity frames: silence means nothing has changed, and treating it as "no
    firmware" would blank the version row sixty times between every pair of updates.

    So it latches on first sight and stays. What it must never do is claim to know something
    it does not: ts == 0 means this parser has never seen the frame at all, and that is the
    only case where fwValid stays False. Firmware older than 625b782a sends 0x707 and not
    0x70F, so the two are tested separately -- a hash with no build flags is the normal
    reading for a board that has not been updated since 2026-09-22.
    """
    if cp.ts_nanos["GW_VERSION"]["GIT_HASH"] != 0:
      # int() is load-bearing: CANParser hands back floats and pycapnp refuses one for a
      # UInt32 field.
      ret_sp.linbusGateway.fwGitHash = int(cp.vl["GW_VERSION"]["GIT_HASH"])
      ret_sp.linbusGateway.fwValid = True

    if cp.ts_nanos["GW_BUILD"]["BUILD_COUNTER"] != 0:
      b = cp.vl["GW_BUILD"]
      ret_sp.linbusGateway.fwDirty = bool(b["BUILD_DIRTY"])
      ret_sp.linbusGateway.fwAppSlot = bool(b["BUILD_APP_SLOT"])
      ret_sp.linbusGateway.fwBootloader = bool(b["BUILD_BOOTLOADER"])
      ret_sp.linbusGateway.fwReadOnly = bool(b["BUILD_READONLY"])
      ret_sp.linbusGateway.boardUid = int(b["BOARD_UID"])
      ret_sp.linbusGateway.fwBuildValid = True

  def _update_linbus_grant(self, ret_sp: structs.CarStateSP, cp: CANParser) -> None:
    """Decode GW_STEER_GRANT (0x70B) from the aftermarket LIN-bus gateway.

    0x704 says whether the board is actuating. This says WHY it is not, which is the only
    thing the driver can act on: "openpilot is asking and the car is not turning" is
    otherwise undiagnosable from the seat.

    THE FRAME MAY NOT EXIST. The board only started sending it in firmware 75aa91ee, and it
    is registered liveness-exempt in get_can_parsers() so its absence cannot cost openpilot
    its CAN. Absence must therefore read as NOT GRANTED -- never as permission -- which is
    what grantValid gating `granted` does. Same frame-counted staleness as GW_ACTIVE, and for
    the same reason: CarState.update() is not handed a clock, and a ts that stops moving is a
    board that stopped talking.
    """
    g = cp.vl["GW_STEER_GRANT"]          # registered liveness-exempt in get_can_parsers()
    ts = cp.ts_nanos["GW_STEER_GRANT"]["STATE"]
    if ts != self._linbus_grant_ts:
      self._linbus_grant_ts = ts
      self._linbus_grant_stale = 0
    else:
      self._linbus_grant_stale = min(self._linbus_grant_stale + 1, LINBUS_GRANT_STALE_FRAMES)

    valid = self._linbus_grant_stale < LINBUS_GRANT_STALE_FRAMES and ts != 0
    state = int(g["STATE"])
    reason = int(g["REASON"])
    retry_in = int(g["RETRY_IN"])
    eps_latched = bool(g["EPS_LATCHED"])

    ret_sp.linbusGateway.grantValid = valid
    ret_sp.linbusGateway.grantState = state if valid else 0
    ret_sp.linbusGateway.grantReason = reason if valid else 0
    ret_sp.linbusGateway.granted = valid and state in GRANT_STATES_STEERING
    ret_sp.linbusGateway.authority = int(g["AUTHORITY"]) if valid else 0
    ret_sp.linbusGateway.epsAck = valid and bool(g["EPS_ACK"])
    # NOT a latch, whatever the signal is called on the wire. Byte 3 bit 1 is the board's
    # `refusing` flag (gw_active.c:1391), which is a TIMED hold: GW_REFUSE_HOLD_MS = 60 s for
    # an EPS error state, but GW_NOACK_HOLD_MS = 3 s for a merely missing acknowledgement
    # (gw_active.c:1017-1022). "The board is inside its refusal hold", nothing stronger.
    ret_sp.linbusGateway.epsLatched = valid and eps_latched
    ret_sp.linbusGateway.epsErrorState = int(g["EPS_ERROR_STATE"]) if valid else 0
    ret_sp.linbusGateway.epsFresh = valid and bool(g["EPS_FRESH"])
    ret_sp.linbusGateway.camLkasOn = valid and bool(g["CAM_LKAS_ON"])
    ret_sp.linbusGateway.applied = int(g["APPLIED"]) if valid else 0
    ret_sp.linbusGateway.motorTorque = int(g["MOTOR_TORQUE"]) if valid else 0
    ret_sp.linbusGateway.retryIn = retry_in if valid else 0
    # RETRY_IN 255 means the EPS has latched until key-off: there is nothing to wait for and
    # nothing to retry into. Reported, never made sticky on this side -- if the board's next
    # fresh frame says otherwise, believe the board, so a one-frame glitch cannot strand the
    # driver for the rest of the drive.
    #
    # RETRY_IN IS THE ONLY AUTHORITY HERE, and EPS_LATCHED is deliberately NOT ORed in. The
    # board emits 255 only while `refusing && eps_errst != 0` (gw_active.c:1401-1406) -- the
    # error-4 case that really does latch for the key cycle. EPS_LATCHED alone is the 3 s
    # no-acknowledgement hold, which this EPS does routinely below about 60 km/h (HANDOFF 0e);
    # ORing it in would tell the driver to cycle the ignition for something that clears itself
    # before he could reach the key.
    ret_sp.linbusGateway.latchedUntilKeyOff = valid and retry_in == GRANT_RETRY_KEY_CYCLE

    # The board's answer has to reach a human somehow. carStateSP is the surface; this is the
    # breadcrumb for the log, one line per change of (state, reason), so a 10 Hz frame cannot
    # flood it.
    key = (valid, state, reason)
    if key != self._linbus_grant_logged:
      self._linbus_grant_logged = key
      if valid and state not in GRANT_STATES_STEERING:
        err = int(g["EPS_ERROR_STATE"])
        carlog.warning(f"LIN-bus gateway not steering: STATE={state} REASON={reason} RETRY_IN={retry_in} EPS_ERR={err}")
