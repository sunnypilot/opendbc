from opendbc.car import CanBusBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.honda.values import (HondaFlags, HONDA_BOSCH_ALT_RADAR, CarControllerParams)
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP

# CAN bus layout with relay
# 0 = ACC-CAN - radar side
# 1 = F-CAN B - powertrain
# 2 = ACC-CAN - camera side
# 3 = F-CAN A - OBDII port


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    # use fingerprint if specified
    super().__init__(CP if fingerprint is None else None, fingerprint)

    # powertrain bus is split instead of radar on radarless and CAN FD Bosch
    if CP is not None and CP.flags & HondaFlags.BOSCH and not (CP.flags & (HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD)):
      self._pt, self._radar = self.offset + 1, self.offset
      # normally steering commands are sent to radar, which forwards them to powertrain bus
      # when radar is disabled, steering commands are sent directly to powertrain bus
      self._lkas = self._pt if CP.openpilotLongitudinalControl else self._radar
    else:
      self._pt, self._radar, self._lkas = self.offset, self.offset + 1, self.offset

  @property
  def pt(self) -> int:
    return self._pt

  @property
  def radar(self) -> int:
    return self._radar

  @property
  def camera(self) -> int:
    return self.offset + 2

  @property
  def lkas(self) -> int:
    return self._lkas

  # B-CAN is forwarded to ACC-CAN radar side (CAN 0 on fake ethernet port)
  @property
  def body(self) -> int:
    return self.offset


# FORK(HONDA_ACCORD_9G_AU): is_metric and elesys are appended, passed by keyword, for the units bit below
def create_brake_command(packer, CAN, apply_brake, pump_on, pcm_override, pcm_cancel_cmd, fcw, stock_brake, CP_SP,
                         is_metric=True, elesys=False):
  # TODO: do we loose pressure if we keep pump off for long?
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False
  # HONDA_ELESYS: this bit is the cluster units flag, 0 = metric / 1 = imperial -- the same
  # meaning as ACC_HUD's IMPERIAL_UNIT below, so it is derived the same way. On every other
  # Nidec Honda it is a reserved constant 1, which is why the shared _nidec_common.dbc still
  # names it SET_ME_1; renaming it there would reach platforms this has not been checked on.
  # is_metric comes from CAR_SPEED.IMPERIAL_UNIT via carstate, i.e. from the cluster itself.
  imperial_unit = int(not is_metric) if elesys else 1

  values = {
    "CRUISE_OVERRIDE": pcm_override,
    "CRUISE_FAULT_CMD": pcm_fault_cmd,
    "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "COMPUTER_BRAKE_REQUEST": brake_rq,
    "SET_ME_1": imperial_unit,   # ELESYS: units flag; reserved 1 elsewhere
    "BRAKE_LIGHTS": brakelights,
    "CHIME": stock_brake["CHIME"] if fcw else 0,  # send the chime for stock fcw
    "FCW": fcw << 1,  # TODO: Why are there two bits for fcw?
    "AEB_REQ_1": 0,
    "AEB_REQ_2": 0,
    "AEB_STATUS": 0
  }

  if CP_SP.flags & HondaFlagsSP.NIDEC_HYBRID:
    values["COMPUTER_BRAKE_HYBRID"] = apply_brake
    values["BRAKE_PUMP_REQUEST_HYBRID"] = apply_brake > 0
  else:
    values["COMPUTER_BRAKE"] = apply_brake
    values["BRAKE_PUMP_REQUEST"] = pump_on

  return packer.make_can_msg("BRAKE_COMMAND", CAN.pt, values)


def create_acc_commands(packer, CAN, enabled, active, accel, gas, stopping_counter, CP):
  commands = []
  min_gas_accel = CarControllerParams.BOSCH_GAS_LOOKUP_BP[0]

  control_on = 5 if enabled else 0
  gas_command = gas if active and accel > min_gas_accel else -30000
  accel_command = accel if active else 0
  braking = 1 if active and accel < min_gas_accel else 0
  standstill = 1 if active and stopping_counter > 0 else 0
  standstill_release = 1 if active and stopping_counter == 0 else 0

  # common ACC_CONTROL values
  acc_control_values = {
    'ACCEL_COMMAND': accel_command,
    'STANDSTILL': standstill,
  }

  if CP.flags & HondaFlags.BOSCH_RADARLESS:
    acc_control_values.update({
      "CONTROL_ON": enabled,
      "IDLESTOP_ALLOW": stopping_counter > 200,  # allow idle stop after 4 seconds (50 Hz)
    })
  else:
    acc_control_values.update({
      # setting CONTROL_ON causes car to set POWERTRAIN_DATA->ACC_STATUS = 1
      "CONTROL_ON": control_on,
      "GAS_COMMAND": gas_command,  # used for gas
      "BRAKE_LIGHTS": braking,
      "BRAKE_REQUEST": braking,
      "STANDSTILL_RELEASE": standstill_release,
    })
    acc_control_on_values = {
      "SET_TO_3": 0x03,
      "CONTROL_ON": enabled,
      "SET_TO_FF": 0xff,
      "SET_TO_75": 0x75,
      "SET_TO_30": 0x30,
    }
    commands.append(packer.make_can_msg("ACC_CONTROL_ON", CAN.pt, acc_control_on_values))

  commands.append(packer.make_can_msg("ACC_CONTROL", CAN.pt, acc_control_values))
  return commands


def create_steering_control(packer, CAN, apply_torque, lkas_active, tja_control,
                            serial_gateway=False, ldw_left=False, ldw_right=False):
  values = {
    "STEER_TORQUE": apply_torque if lkas_active else 0,
    "STEER_TORQUE_REQUEST": lkas_active,
  }

  if tja_control:
    values["STEER_DOWN_TO_ZERO"] = lkas_active

  # FORK(HONDA_ELESYS): nothing in this car reads 0x0E4 -- the EPS has no CAN steering input
  # and the in-line LIN-bus gateway board consumes the frame and re-emits it on the camera's
  # serial line. Byte 2 bits 5:4 are SPECIFIED (SP-PROTOCOL-V3 section 4) to reach serial
  # camera-to-EPS byte 2 bits 5:4, where the stock camera puts its lane-departure warning.
  #
  # THEY ARE INERT ON FIRMWARE 875ba124, in both directions. gw_active.c:759-773 is the whole
  # of the board's 0x0E4 parse and reads only bit 7 (the request) and bit 2 (the domain), and
  # lkas_uart.c:449 builds serial byte 2 as 0x80 | (lkas_on ? 0 : 0x40), so bits 5:4 are hard
  # zero on every frame the board transmits. The 0x500 LDW_ACTIVE path is the same: sp_hud.c
  # decodes it, sp_hud_merge_lkas() never reads it. Sent regardless, because they cost nothing
  # and let the board-side change land without another openpilot commit -- but do not go
  # looking for a cluster warning yet, and do not read a correct 0x0E4 in the log as proof the
  # EPS saw anything.
  #
  # Only these two bits are filled. Byte 2 bit 2 is the board's SERIAL_DOMAIN declaration and
  # MUST stay clear while openpilot is in the 2560 CAN domain: setting it tells the board to
  # take STEER_TORQUE as serial counts at unity gain, which pins it at full authority from the
  # first frame. It is held at zero by SET_ME_X00_3 in the DBC, along with bits 3, 1 and 0.
  # The signals exist only in honda_accord_au_2015_can; guard, or the packer logs an unknown
  # signal on every other Honda.
  if serial_gateway:
    values["LDW_RIGHT"] = ldw_right
    values["LDW_LEFT"] = ldw_left

  return packer.make_can_msg("STEERING_CONTROL", CAN.lkas, values)


def create_bosch_supplemental_1(packer, CAN):
  # non-active params
  values = {
    "SET_ME_X04": 0x04,
    "SET_ME_X80": 0x80,
    "SET_ME_X10": 0x10,
  }
  return packer.make_can_msg("BOSCH_SUPPLEMENTAL_1", CAN.lkas, values)


def create_acc_hud(packer, bus, CP, enabled, pcm_speed, pcm_accel, hud_control, hud_v_cruise, is_metric, acc_hud):
  acc_hud_values = {
    'CRUISE_SPEED': hud_v_cruise,
    'ENABLE_MINI_CAR': 1 if enabled else 0,
    # only moves the lead car without ACC_ON
    'HUD_DISTANCE': hud_control.leadDistanceBars,  # wraps to 0 at 4 bars
    'IMPERIAL_UNIT': int(not is_metric),
    'HUD_LEAD': 2 if enabled and hud_control.leadVisible else 1 if enabled else 0,
    'SET_ME_X01_2': 1,
  }

  if CP.flags & HondaFlags.BOSCH:
    acc_hud_values['ACC_ON'] = int(enabled)
    acc_hud_values['FCM_OFF'] = 1
    acc_hud_values['FCM_OFF_2'] = 1
  else:
    # Shows the distance bars, TODO: stock camera shows updates temporarily while disabled
    acc_hud_values['ACC_ON'] = int(enabled)
    acc_hud_values['PCM_SPEED'] = pcm_speed * CV.MS_TO_KPH
    acc_hud_values['PCM_GAS'] = pcm_accel
    acc_hud_values['SET_ME_X01'] = 1
    acc_hud_values['FCM_OFF'] = acc_hud['FCM_OFF']
    acc_hud_values['FCM_OFF_2'] = acc_hud['FCM_OFF_2']
    acc_hud_values['FCM_PROBLEM'] = acc_hud['FCM_PROBLEM']
    acc_hud_values['ICONS'] = acc_hud['ICONS']

  return packer.make_can_msg("ACC_HUD", bus, acc_hud_values)


def create_scm_buttons_no_cruise(packer, bus, scm_buttons):
  # Re-send SCM_BUTTONS to the radar with the master ACC switch forced OFF (MAIN_ON=0), continuously,
  # so the stock ACC (Elesys radar) stands down (OP replaces it). The radar reads MAIN from this message; CMBS
  # is independent of MAIN and keeps working. The PCM on the pt bus still gets the driver's real
  # buttons/MAIN, so OP engages normally. Copy every signal except CHECKSUM/COUNTER (packer redoes).
  values = {s: scm_buttons[s] for s in scm_buttons if s not in ("CHECKSUM", "COUNTER")}
  values["MAIN_ON"] = 0        # master ACC switch off -> stock ACC stands down
  values["CRUISE_BUTTONS"] = 0  # neutralize any in-flight cruise button
  return packer.make_can_msg("SCM_BUTTONS", bus, values)


def create_lkas_hud(packer, bus, CP, hud_control, lat_active, steering_available, alert_steer_required, lkas_hud, dashed_lanes):
  commands = []

  lkas_hud_values = {
    'LKAS_READY': 1,
    'LKAS_STATE_CHANGE': 1,
    'STEERING_REQUIRED': alert_steer_required,
    'SOLID_LANES': lat_active,
    'DASHED_LANES': dashed_lanes,
    'BEEP': 0,
  }

  if CP.flags & (HondaFlags.BOSCH_RADARLESS | HondaFlags.BOSCH_CANFD):
    lkas_hud_values['LANE_LINES'] = 3
    lkas_hud_values['DASHED_LANES'] = lat_active

    # car likely needs to see LKAS_PROBLEM fall within a specific time frame, so forward from camera
    # TODO: needed for Bosch CAN FD?
    if CP.flags & HondaFlags.BOSCH_RADARLESS:
      lkas_hud_values['LKAS_PROBLEM'] = lkas_hud['LKAS_PROBLEM']

  if not (CP.flags & HondaFlags.BOSCH_EXT_HUD):
    lkas_hud_values['RDM_OFF'] = 1
    lkas_hud_values['LANE_ASSIST_BEEP_OFF'] = 1

  # New HUD concept for selected Bosch cars, overwrites some of the above
  # TODO: make global across all Honda if feedback is favorable
  if CP.carFingerprint in HONDA_BOSCH_ALT_RADAR:
    lkas_hud_values['DASHED_LANES'] = steering_available and lat_active
    lkas_hud_values['SOLID_LANES'] = lat_active

  if CP.flags & HondaFlags.BOSCH_EXT_HUD and not CP.openpilotLongitudinalControl:
    commands.append(packer.make_can_msg('LKAS_HUD_A', bus, lkas_hud_values))
    commands.append(packer.make_can_msg('LKAS_HUD_B', bus, lkas_hud_values))
  else:
    commands.append(packer.make_can_msg('LKAS_HUD', bus, lkas_hud_values))

  return commands


def create_radar_hud(packer, bus):
  radar_hud_values = {
    'CMBS_OFF': 0x01,
    'SET_TO_1': 0x01,
  }

  return packer.make_can_msg('RADAR_HUD', bus, radar_hud_values)


def create_legacy_brake_command(packer, bus):
  return packer.make_can_msg("LEGACY_BRAKE_COMMAND", bus, {})


def spam_buttons_command(packer, CAN, button_val, CP):
  values = {
    'CRUISE_BUTTONS': button_val,
    'CRUISE_SETTING': 0,
  }
  # send buttons to camera on radarless (camera does ACC) cars
  bus = CAN.camera if CP.flags & HondaFlags.BOSCH_RADARLESS else CAN.pt
  return packer.make_can_msg("SCM_BUTTONS", bus, values)


SP_HUD_PROTOCOL_VERSION = 3

# v3 byte 5 OP_STATE. openpilot's OWN lateral state, not the board's -- the board reports its
# state on GW_STEER_GRANT (0x70B) STATE.
SP_OP_STATE_OFF = 0
SP_OP_STATE_READY = 1
SP_OP_STATE_REQUESTING = 2
SP_OP_STATE_ACTIVE = 3
SP_OP_STATE_WITHDRAWING = 4
SP_OP_STATE_FAULTED = 5

# v3 byte 6 MAX_TORQUE, in SERIAL counts. 0 means "use your own authority", and that is what
# this sends on purpose: the board scales openpilot's 2560-count full scale by
# authority/2560, so openpilot's full scale already IS the board's authority whatever the
# authority is. Sending the number here as well would put the authority ladder
# (40 -> 80 -> 120 -> 160) in two places that could disagree.
#
# THE FIELD IS REPORTED, NOT ENFORCED, ON FIRMWARE 875ba124. The board reads it in exactly one
# place -- gw_active.c:1348 -- and only to compute the AUTHORITY byte it puts in 0x70B. The
# command path clamps to the compile-time GW_LIN_AUTHORITY unconditionally
# (gw_active.c:1104,1108), so a non-zero value here would make 0x70B report a cap the board
# was not applying. That costs nothing while this is 0, which is the other reason it is 0.
# Do not use it as a probe-drive limiter until the board clamps to the negotiated value.
SP_HUD_MAX_TORQUE = 0


def create_sp_hud_status(packer, bus, CC, CC_SP, hud_control, alert_steer_required, alert_fcw,
                         lat_ready, op_state, release_brake, release_driver):
  """openpilot's alert state, for an aftermarket module sitting in line with the LKAS camera.

  This is NOT a stock Honda message and nothing in the car reads it. It exists so a module
  that is already passing the camera's LKAS_HUD through can merge openpilot's alerts into
  that frame, instead of openpilot taking 0x33D over itself. Taking 0x33D over would drop
  the camera's RDM_HUD lane-departure popup, which openpilot has no road-departure logic to
  reproduce, and would blind carstate's LKAS_PROBLEM check -- on HONDA_ELESYS that reads
  0x33D off bus 0, so openpilot would end up reading back its own frame.

  CHECKSUM and COUNTER are filled by the packer, because they are named exactly that in a
  honda_ DBC. See _sunnypilot_linbus_gw.dbc.

  THE VERSION NUMBER IS A HARD GATE ON THE RECEIVER. The board rejects a 0x500 whose version
  is above the maximum it knows (sp_hud.c sp_hud_rx / SP_HUD_VERSION_MAX), and rejecting the
  frame silently takes the HUD merge and the board's integrator guard down with it. Board
  firmware 75aa91ee is the first that accepts 3. Never raise SP_HUD_PROTOCOL_VERSION ahead of
  the flashed image. The board also drops the frame on a bad Honda checksum or on a COUNTER
  that has stopped moving, so neither may be hand-filled here.

  CC_SP.lateralControl is a DATACLASS, not a dict: convert_carControlSP() in
  selfdrive/car/helpers.py rebuilds every nested struct by hand. It did not rebuild
  lateralControl when that was added, so this function raised AttributeError on every control
  step and took card down with it (routes 000000b5-b8). Attribute access below is correct
  BECAUSE of that rebuild; test_car_control_sp_seam.py guards the seam.
  """
  # 3 critical, 2 warning, 1 info, 0 none. A severity hint only -- which alert it is lives
  # in the flag bits, so a receiver never has to infer one from the other.
  if alert_fcw:
    alert_level = 3
  elif alert_steer_required or hud_control.leftLaneDepart or hud_control.rightLaneDepart:
    alert_level = 2
  elif CC.enabled:
    alert_level = 1
  else:
    alert_level = 0

  values = {
    'PROTOCOL_VERSION': SP_HUD_PROTOCOL_VERSION,
    'OP_ENABLED': CC.enabled,
    'LAT_ACTIVE': CC.latActive,
    'LONG_ACTIVE': CC.longActive,
    'STEERING_REQUIRED': alert_steer_required,
    'LDW_LEFT': hud_control.leftLaneDepart,
    'LDW_RIGHT': hud_control.rightLaneDepart,
    'FCW': alert_fcw,
    'SOLID_LANES': CC.latActive,
    'DASHED_LANES': hud_control.lanesVisible and not CC.latActive,
    'LEAD_VISIBLE': hud_control.leadVisible,
    'ALERT_LEVEL': alert_level,
    # ALWAYS km/h, never the user's display units. hud_v_cruise in carcontroller.py is
    # setSpeed / v_cruise_factor, which is mph when the cluster is imperial -- putting that
    # on the wire would make the signal mean two different things depending on a setting the
    # receiver cannot see. Converted here so the DBC unit is simply true.
    # 0 means "no set speed available"; the value saturates rather than wrapping, so a
    # receiver never sees a plausible-but-wrong low speed.
    'SET_SPEED': max(0, min(255, int(hud_control.setSpeed * CV.MS_TO_KPH))) if hud_control.speedVisible else 0,
    # v2: the lateral integrator, so the gateway can refuse its first engagement while
    # |i| is large (it cannot see i from STEERING_CONTROL, and cannot infer it from
    # saturation -- openpilot was NOT saturated in the curve where i sat at +0.65).
    # int8, x100, clipped: +-1.27 covers the PID's whole range on this car.
    'INTEGRATOR': max(-127, min(127, int(round(CC_SP.lateralControl.integrator * 100)))),
    'OP_SATURATED': CC_SP.lateralControl.saturated,
    # the acknowledgement: 1 while the gateway hold on the integrator is active. If the
    # board sees this low while it is not engaged, sunnypilot is not running the protocol.
    'INTEGRATOR_FROZEN': CC_SP.lateralControl.integratorFrozen,
    # v3, bytes 5-6: the control request of SP-PROTOCOL-V3 section 1.2. Advisory -- 0x0E4
    # byte 2 bit 7 remains the thing that actually asks for torque, and every gate on the
    # board still applies. This says what openpilot INTENDS, so the board can answer why it
    # is not steering in openpilot's own terms on 0x70B.
    'WANT_CONTROL': CC.latActive,
    'LAT_READY': lat_ready,
    'OP_STATE': op_state,
    'RELEASE_BRAKE': release_brake,
    'RELEASE_DRIVER': release_driver,
    'LDW_ACTIVE': hud_control.leftLaneDepart or hud_control.rightLaneDepart,
    'MAX_TORQUE': SP_HUD_MAX_TORQUE,
  }
  return packer.make_can_msg("SP_HUD_STATUS", bus, values)


def honda_checksum(address: int, sig, d: bytearray) -> int:
  s = 0
  extended = address > 0x7FF
  addr = address
  while addr:
    s += addr & 0xF
    addr >>= 4
  for i in range(len(d)):
    x = d[i]
    if i == len(d) - 1:
      x >>= 4
    s += (x & 0xF) + (x >> 4)
  s = 8 - s
  if extended:
    s += 3
  return s & 0xF
