#!/usr/bin/env python3
"""Integration checks: the real Honda ELESYS CarController, driven frame by frame
with the dynamic tuner switched on.

test_dynamic_tuning.py covers HondaDynamicTuner in isolation. It cannot catch the
class of bug that lives in the *seam* -- call ordering, the 100 Hz / 50 Hz / 10 Hz
rate split, whether a learned gain reaches an
actuator it was never meant to touch. That is what this file is for, so every check
below goes through CarController.update() and reads what would go on the wire.

Runs standalone: PYTHONPATH=<opendbc_repo> python this_file.py
"""

import math
import sys
from dataclasses import dataclass, field

import numpy as np

from opendbc.car import Bus, structs
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.carcontroller import CarController, brake_release_scale, BRAKE_RELEASE_FRAMES
from opendbc.car.honda.hondacan import honda_checksum
from opendbc.car.honda.values import CAR, DBC, CarControllerParams
from opendbc.can.packer import CANPacker
from opendbc.can.dbc import DBC as DBCFile
from opendbc.sunnypilot.car.honda import dynamic_tuning as dt

LongCtrlState = structs.CarControl.Actuators.LongControlState

PLATFORM = CAR.HONDA_ACCORD_9G_AU     # the HONDA_ELESYS platform
FAILURES = []


def check(name, cond, detail=""):
  print(("  PASS  " if cond else "  FAIL  ") + name + ("" if cond else f"  {detail}"))
  if not cond:
    FAILURES.append(name)


# --- a Params double that enforces the real contract -------------------------

@dataclass
class FakeParams:
  """Mirrors the parts of the contract that matter: unknown keys raise (as
  check_key does), and put() only accepts an exact float for a FLOAT key (as
  python2cpp's (type(dat), t) lookup does)."""
  store: dict = field(default_factory=dict)
  writes: int = 0
  bad_writes: list = field(default_factory=list)

  def _spec(self, key):
    if key in ("HondaDynamicTuningEnabled",):
      return None
    if key not in dt._PARAM_SPEC:
      raise KeyError(key)
    return dt._PARAM_SPEC[key]

  def get(self, key, block=False, return_default=False):
    spec = self._spec(key)
    return self.store.get(key, spec[0] if spec else None)

  def get_bool(self, key, block=False):
    return bool(self.store.get(key, False))

  def put(self, key, val, block=False):
    self._spec(key)
    if type(val) is not float:
      self.bad_writes.append((key, type(val).__name__))
    self.writes += 1
    self.store[key] = val


def build(tuning=True):
  params = FakeParams()
  params.store["HondaDynamicTuningEnabled"] = tuning
  dt._open_params = lambda: params

  CP = CarInterface.get_non_essential_params(PLATFORM)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, PLATFORM)
  CP.openpilotLongitudinalControl = True
  CP_SP.enableGasInterceptor = True
  return CarController(PLATFORM.config.dbc_dict, CP, CP_SP), CP, CP_SP, params


PARAMS = CarControllerParams(CarInterface.get_non_essential_params(PLATFORM))
CC_SP = structs.CarControlSP()


def make_cc(accel, state=LongCtrlState.pid, long_active=True, pitch=0.0):
  cc = structs.CarControl.new_message()
  cc.enabled = long_active
  cc.longActive = long_active
  cc.orientationNED = [0.0, pitch, 0.0]
  cc.actuators.accel = accel
  cc.actuators.longControlState = state
  cc.hudControl.speedVisible = True
  cc.hudControl.setSpeed = 30.0
  return cc.as_reader()


class CS:
  def __init__(self):
    self.out = structs.CarState.new_message()
    self.out.cruiseState.speed = 30.0
    self.out.cruiseState.available = True
    self.v_cruise_factor = 1.0
    self.stock_brake = {"CHIME": 0, "AEB_REQ_1": 0, "AEB_REQ_2": 0, "AEB_STATUS": 0}
    self.acc_hud = {"FCM_OFF": 0, "FCM_OFF_2": 0, "FCM_PROBLEM": 0, "ICONS": 0}
    self.lkas_hud = {}
    self.scm_buttons = {"CRUISE_BUTTONS": 0, "CRUISE_SETTING": 0}
    self.is_metric = True


ACC_HUD_ADDR = 0x30C


def acc_hud_pcm_gas(can_sends):
  """PCM_GAS as it actually reaches the wire: byte 2 of ACC_HUD.

  Read off the CAN frames rather than off the tuner, because the tuner no longer has
  a PCM channel to ask -- and what matters is that nothing puts gas in this byte for
  an interceptor car, whatever the internals do.
  """
  out = 0
  for m in can_sends:
    # packer.make_can_msg returns a plain (address, dat, bus) tuple; CanData objects
    # carry the same fields by name. Accept either so this does not depend on which.
    addr = m[0] if isinstance(m, tuple) else m.address
    dat = m[1] if isinstance(m, tuple) else m.dat
    if addr == ACC_HUD_ADDR and len(dat) > 2:
      out = max(out, dat[2])
  return out


def drive(cc_obj, phases, plant_gain=0.75, tau=0.30):
  """Run a scripted drive with a crude first-order plant. Returns per-frame traces."""
  cs = CS()
  v = a = 0.0
  trace = []
  for i, (target, state, pitch) in enumerate(phases):
    long_active = state != LongCtrlState.off
    a += (plant_gain * target - a) * (0.01 / tau)
    v = max(0.0, v + a * 0.01)
    cs.out.vEgo, cs.out.aEgo = v, a
    cs.out.standstill = v < 0.01
    _, sends = cc_obj.update(make_cc(target, state, long_active, pitch), CC_SP, cs, i * int(1e7))
    trace.append({"i": i, "v": v, "brake": cc_obj.apply_brake_last, "gas": cc_obj.gas,
                  "pcm": acc_hud_pcm_gas(sends), "state": state,
                  "gain": 1.0 + cc_obj.dynamic_tuner.brake_pid_factor})
  return trace


# --- 1. the toggle off must be bit-identical to stock ------------------------

print("\n[1] toggle off == stock")
PHASES = ([(0.9, LongCtrlState.pid, 0.0)] * 400 + [(0.2, LongCtrlState.pid, 0.03)] * 600 +
          [(-1.5, LongCtrlState.pid, -0.03)] * 500 + [(-0.8, LongCtrlState.stopping, 0.0)] * 400)
off, *_ = build(tuning=False)
t_off = drive(off, PHASES)
check("tuner reports disabled", not off.dynamic_tuner.enabled)
check("no PCM gas is ever requested with the interceptor",
      all(f["pcm"] == 0 for f in t_off))
check("brake gain is exactly 1.0 every frame",
      all(abs(f["gain"] - 1.0) < 1e-12 for f in t_off))

# same drive with the tuner on: nothing may leave the legal range
print("\n[2] toggle on")
on, CP, CP_SP, params = build()
t_on = drive(on, PHASES)
check("tuner came up", on.dynamic_tuner.enabled)
check("brake command stays in range",
      all(0 <= f["brake"] <= PARAMS.NIDEC_BRAKE_MAX - 1 for f in t_on),
      f"{min(f['brake'] for f in t_on)}..{max(f['brake'] for f in t_on)}")
check("interceptor command stays in range", all(0.0 <= f["gas"] <= 1.0 for f in t_on))
check("PCM gas stays at zero with the tuner on too", all(f["pcm"] == 0 for f in t_on))
check("no learned value goes non-finite",
      all(math.isfinite(x) for k, x in on.dynamic_tuner.debug_values().items()
          if isinstance(x, float)))
check("every param key resolves and every write is an exact float",
      on.dynamic_tuner.debug_values()["write_errors"] == 0 and not params.bad_writes,
      f"errors={on.dynamic_tuner.debug_values()['write_errors']} bad={params.bad_writes}")


# --- 3. gas and brake can never be commanded together ------------------------

print("\n[3] no concurrent gas + brake")
cc_obj, *_ = build()
cs = CS()
bad = []
bad_pcm = []
v = 6.0
for i in range(1200):
  cs.out.vEgo, cs.out.aEgo = v, -1.0
  _, sends = cc_obj.update(make_cc(-2.0), CC_SP, cs, i * int(1e7))
  if cc_obj.apply_brake_last > 0 and cc_obj.gas > 0.0:
    bad.append(i)
  if cc_obj.apply_brake_last > 0 and acc_hud_pcm_gas(sends) > 0:
    bad_pcm.append(i)
  v = max(0.0, v - 0.005)
check("interceptor gas is cut whenever brake is commanded", not bad, f"{len(bad)} frames")
check("no PCM gas while braking either", not bad_pcm, f"{len(bad_pcm)} frames")


# --- 4. the standstill hold is exactly what interface.py asked for -----------
#
# Regression: the learned brake gain multiplies the same command stopAccel was
# hand-tuned against. Measured, a converged gain of 0.33 put the hold back at
# cb 251 and 0.50 railed it -- silently undoing ret.stopAccel = -0.8.

print("\n[4] standstill hold is not scaled by the learned gain")
holds = {}
for gain in (0.0, 0.25, 0.5, 0.6):
  cc_obj, *_ = build()
  cc_obj.dynamic_tuner.brake_gain_converged = gain
  cc_obj.dynamic_tuner.brake_pid.i = gain
  cc_obj.dynamic_tuner.brake_pid_factor = gain
  cs = CS()
  cs.out.vEgo = cs.out.aEgo = 0.0
  cs.out.standstill = True
  for i in range(600):
    cc_obj.update(make_cc(CP.stopAccel, LongCtrlState.stopping), CC_SP, cs, i * int(1e7))
  holds[gain] = cc_obj.apply_brake_last
print(f"        hold command by converged gain: {holds}")
check("the hold is identical at every learned gain", len(set(holds.values())) == 1, str(holds))
check("and it is well clear of the rail", max(holds.values()) < PARAMS.NIDEC_BRAKE_MAX - 20,
      f"{max(holds.values())} of {PARAMS.NIDEC_BRAKE_MAX - 1}")


# --- 5. a disengage does not carry a wound gain into the next engagement -----

print("\n[5] disengage unwinds the brake gain")
cc_obj, *_ = build()
cc_obj.dynamic_tuner.brake_pid.i = dt.BRAKE_POS_LIMIT
cc_obj.dynamic_tuner.brake_pid_factor = dt.BRAKE_POS_LIMIT
cs = CS()
cs.out.vEgo, cs.out.aEgo = 15.0, 0.0
for i in range(200):                                     # disengaged
  cc_obj.update(make_cc(0.0, LongCtrlState.off, long_active=False), CC_SP, cs, i * int(1e7))
cc_obj.update(make_cc(-0.5), CC_SP, cs, 200 * int(1e7))  # re-engage
check("the integrator is back at the converged estimate on re-engage",
      abs(cc_obj.dynamic_tuner.brake_pid_factor - cc_obj.dynamic_tuner.brake_gain_converged) < 1e-9,
      f"{cc_obj.dynamic_tuner.brake_pid_factor:.4f}")


# --- 6. the interceptor owns the gas at every speed ---------------------------
#
# There used to be a pedal/PCM crossfade here that handed part of the request back
# to the PCM above ~30 km/h. It is gone: the PCM was never once shown to answer
# openpilot's ACC_HUD, and it is the harder actuator to control anyway. What is
# left to prove is the inverse -- that nothing puts gas in the PCM byte at any
# speed, and that the interceptor command does not fall away as speed rises.

print("\n[6] interceptor owns the gas at every speed")
cc_obj, *_ = build()
cs = CS()
seen = {}
pcm_any = 0
for i, v in enumerate(np.linspace(0.0, 25.0, 4000)):
  cs.out.vEgo, cs.out.aEgo = float(v), 0.5
  _, sends = cc_obj.update(make_cc(1.0), CC_SP, cs, i * int(1e7))
  pcm_any = max(pcm_any, acc_hud_pcm_gas(sends))
  seen[round(float(v), 1)] = cc_obj.gas
check("PCM gas is zero at every speed", pcm_any == 0, f"max {pcm_any}")
lo = [g for v, g in seen.items() if v < 8.0]
hi = [g for v, g in seen.items() if v > 15.0]
check("the interceptor still carries the request at high speed", max(hi) > 0.0, f"max hi {max(hi):.3f}")
check("no crossfade cliff: high-speed command is not cut below the low-speed one",
      max(hi) >= max(lo) - 1e-9, f"hi {max(hi):.3f} vs lo {max(lo):.3f}")


# --- 7. SP_HUD_STATUS reaches the wire ----------------------------------------
#
# openpilot does not send LKAS_HUD on this car -- the stock camera keeps 0x33D so its
# RDM_HUD lane-departure popup and LKAS_PROBLEM survive. openpilot's own alert state goes
# out on 0x500 instead, for an in-line module to merge. This guards the three things that
# have to line up for that to work: the DBC entry, the call site, and the panda allowlist.

print("\n[7] SP_HUD_STATUS side channel")
cc_obj, *_ = build()
cs = CS()
cs.out.vEgo, cs.out.aEgo = 20.0, 0.0
sp_frames = []
lkas_frames = []
for i in range(60):
    _, sends = cc_obj.update(make_cc(0.5), CC_SP, cs, i * int(1e7))
    for m in sends:
        addr = m[0] if isinstance(m, tuple) else m.address
        dat = m[1] if isinstance(m, tuple) else m.dat
        bus = m[2] if isinstance(m, tuple) else m.src
        if addr == 0x500:
            sp_frames.append((bus, bytes(dat)))
        if addr == 0x33D:
            lkas_frames.append(addr)

check("SP_HUD_STATUS is transmitted", len(sp_frames) > 0, f"{len(sp_frames)} frames")
check("openpilot still does NOT send LKAS_HUD on this car", not lkas_frames,
      f"{len(lkas_frames)} frames of 0x33D")
if sp_frames:
    bus, dat = sp_frames[0]
    # bus 0, not 2: bus 2 is the Elesys radar branch on this harness and the gateway board
    # never sees it (route 000000b9: 0x500 only ever on src 130)
    check("on bus 0, 8 bytes", bus == 0 and len(dat) == 8, f"bus={bus} dlc={len(dat)}")
    bad = [d for _, d in sp_frames if (d[7] & 0x0F) != honda_checksum(0x500, None, bytearray(d))]
    check("every frame carries a valid Honda checksum", not bad, f"{len(bad)} bad")
    # v3 since the SP-PROTOCOL-V3 round. The board REJECTS an unknown version whole
    # (sp_hud.c SP_HUD_VERSION_MAX), which silently takes the HUD merge and the integrator
    # guard with it, so this number must never lead the flashed firmware.
    check("protocol version is 3", (dat[0] >> 4) & 0x0F == 3, f"{(dat[0] >> 4) & 0x0F}")
    # set speed is km/h on the wire regardless of cluster units: 30 m/s -> 108
    check("SET_SPEED is km/h, not the cluster's display units", dat[2] == 108, f"{dat[2]}")
    counters = [(d[7] >> 4) & 0x03 for _, d in sp_frames]
    check("COUNTER advances", len(set(counters)) > 1, f"{sorted(set(counters))}")

    # v2: the lateral integrator and its flags come from CarControlSP.lateralControl
    def sp_frame(cc_sp, start):
      out = None
      for j in range(start, start + 20):
        _, s = cc_obj.update(make_cc(0.5), cc_sp, cs, j * int(1e7))
        for m in s:
          if (m[0] if isinstance(m, tuple) else m.address) == 0x500:
            out = bytes(m[1] if isinstance(m, tuple) else m.dat)
      return out
    cc_sp_v2 = structs.CarControlSP()
    cc_sp_v2.lateralControl.integrator = 0.65
    cc_sp_v2.lateralControl.saturated = True
    cc_sp_v2.lateralControl.integratorFrozen = True
    f = sp_frame(cc_sp_v2, 100)
    check("v2 INTEGRATOR is i*100 as int8 (+0.65 -> 65)",
          f is not None and int.from_bytes(f[3:4], "big", signed=True) == 65, f"{f and f[3]}")
    check("v2 OP_SATURATED (b4.0) and INTEGRATOR_FROZEN (b4.1) set", f is not None and (f[4] & 0x03) == 0x03, f"{f and f[4]:#04x}")
    cc_sp_v2.lateralControl.integrator = -0.42
    cc_sp_v2.lateralControl.saturated = False
    cc_sp_v2.lateralControl.integratorFrozen = False
    f = sp_frame(cc_sp_v2, 200)
    check("v2 INTEGRATOR negative (-0.42 -> -42)",
          f is not None and int.from_bytes(f[3:4], "big", signed=True) == -42, f"{f and f[3]}")
    check("v2 flags clear", f is not None and (f[4] & 0x03) == 0, f"{f and f[4]:#04x}")
    cc_sp_v2.lateralControl.integrator = 9.9
    f = sp_frame(cc_sp_v2, 300)
    check("v2 INTEGRATOR clips at +127 rather than wrapping",
          f is not None and int.from_bytes(f[3:4], "big", signed=True) == 127, f"{f and f[3]}")
    check("v2 frames still carry a valid checksum",
          f is not None and (f[7] & 0x0F) == honda_checksum(0x500, None, bytearray(f)))


# --- 8. GW_ACTIVE reaches CarStateSP.linbusGateway ------------------------------
#
# The other direction of the protocol. The LIN-bus gateway reports whether it is
# actually actuating; carstate has to turn that into the one flag the lateral
# controller keys off, and a gateway that goes quiet has to read as NOT actuating,
# or the integrator winds up against a car that stopped listening.

print("\n[8] GW_ACTIVE -> CarStateSP.linbusGateway")
CP8 = CarInterface.get_non_essential_params(PLATFORM)
CP8_SP = CarInterface.get_non_essential_params_sp(CP8, PLATFORM)
CI8 = CarInterface(CP8, CP8_SP)


def gw_frame(engaged, dry_run):
  d = bytearray(8)
  if dry_run:
    d[5] |= 0x80      # DRY_RUN: byte 5 bit 7
  if engaged:
    d[6] |= 0x01      # ENGAGED: byte 6 bit 0
  return (0x704, bytes(d), 0)


def gw_step(i, frames):
  # +1: a packet timestamp of exactly 0 is the parser's "never received" sentinel, which
  # carstate deliberately reads as stale. Real timestamps are monotonic nanos, never 0.
  _, cs_sp = CI8.update([((i + 1) * int(1e7), frames)])
  return cs_sp.linbusGateway


g = gw_step(0, [gw_frame(0, 0)])
check("present is set on this platform", g.present)
check("a frame arrived -> valid", g.valid)
check("not engaged -> not actuating", not g.actuating)
g = gw_step(1, [gw_frame(1, 1)])
check("engaged in a DRY RUN is NOT actuating", g.engaged and g.dryRun and not g.actuating)
g = gw_step(2, [gw_frame(1, 0)])
check("engaged and not dry-run IS actuating", g.actuating)
# the board goes quiet: must read as not actuating within the 500 ms window
for i in range(3, 3 + 49):
  g = gw_step(i, [])
check("still actuating one frame inside the window", g.valid and g.actuating)
g = gw_step(52, [])
check("stale after 50 frames with no GW_ACTIVE -> not actuating", not g.valid and not g.actuating)
g = gw_step(53, [gw_frame(1, 0)])
check("recovers the frame the board speaks again", g.valid and g.actuating)

# An absent board must NOT cost openpilot its CAN. GW_ACTIVE is registered liveness-exempt:
# with no frame ever received, its MessageState still reports valid, so it can never be the
# reason can_valid goes false and engagement is refused.
st = CI8.can_parsers[Bus.pt].message_states[0x704]
check("GW_ACTIVE is registered before the first update", 0x704 in CI8.can_parsers[Bus.pt].addresses)
check("GW_ACTIVE is liveness-exempt (ignore_alive)", st.ignore_alive)
fresh_ci = CarInterface(CP8, CP8_SP)
fresh_ci.update([(int(1e7), [])])
fresh_st = fresh_ci.can_parsers[Bus.pt].message_states[0x704]
check("a board that never speaks still reads as a valid message, not a CAN timeout",
      fresh_st.valid(int(5e9), False) and not fresh_st.timestamps)


# --- 9. Fuel level and odometer reach CarState -----------------------------------
#
# Reverse-engineered from 38 routes (see _nidec_scm_group_a_elesys.dbc). This checks the
# DBC layout and the carstate clip, using the real packer so the checksum and counter are
# what the parser expects.

print("\n[9] SCM_BUTTONS.FUEL_LEVEL -> CarState.fuelGauge, SCM_FEEDBACK.ODOMETER_KM")
CP9 = CarInterface.get_non_essential_params(PLATFORM)
CP9_SP = CarInterface.get_non_essential_params_sp(CP9, PLATFORM)
CI9 = CarInterface(CP9, CP9_SP)
packer9 = CANPacker(DBC[PLATFORM][Bus.pt])


def as_tuple(m):
  return m if isinstance(m, tuple) else (m.address, bytes(m.dat), m.src)


def fuel_step(i, level, sender=0, odo=164964):
  frames = [as_tuple(packer9.make_can_msg("SCM_BUTTONS", 0, {"FUEL_LEVEL": level, "FUEL_SENDER": sender, "MAIN_ON": 1})),
            as_tuple(packer9.make_can_msg("SCM_FEEDBACK", 0, {"ODOMETER_KM": odo}))]
  cs9, _ = CI9.update([((i + 1) * int(1e7), frames)])
  return cs9


vl9 = CI9.can_parsers[Bus.pt].vl
# The parser registers a message the first time carstate reads it (lazy VLDict), which is
# after the first cp.update() -- so the very first frame of any message is dropped, on the car
# (40 ms at 25 Hz) exactly as here. One warm-up step, then the real checks.
fuel_step(0, 0)
cs9 = fuel_step(1, 19, 177)
check("FUEL_LEVEL 19 -> fuelGauge 19/105", abs(cs9.fuelGauge - 19 / 105) < 1e-6, f"{cs9.fuelGauge:.4f}")
check("the packed frame round-trips FUEL_LEVEL (byte 3) and FUEL_SENDER (byte 4)",
      vl9["SCM_BUTTONS"]["FUEL_LEVEL"] == 19 and vl9["SCM_BUTTONS"]["FUEL_SENDER"] == 177)
check("FUEL_LEVEL does not disturb the button decode that shares the frame",
      vl9["SCM_BUTTONS"]["MAIN_ON"] == 1 and vl9["SCM_BUTTONS"]["CRUISE_BUTTONS"] == 0)
cs9 = fuel_step(2, 105, 46)
check("the meter's clamp value 105 reads as a full gauge", abs(cs9.fuelGauge - 1.0) < 1e-6, f"{cs9.fuelGauge:.4f}")
cs9 = fuel_step(3, 130, 22)
check("above the clamp is clipped to 1.0, never >1", cs9.fuelGauge == 1.0, f"{cs9.fuelGauge:.4f}")
check("ODOMETER_KM is the 24-bit field at bytes 3-5", vl9["SCM_FEEDBACK"]["ODOMETER_KM"] == 164964,
      f"{vl9['SCM_FEEDBACK']['ODOMETER_KM']}")
raw9 = bytes(as_tuple(packer9.make_can_msg("SCM_FEEDBACK", 0, {"ODOMETER_KM": 164964}))[1])
check("...and byte 3 carries the top byte (2), matching the car", raw9[3] == 2 and raw9[4] == 0x84 and raw9[5] == 0x64, raw9.hex())


# --- 10. SP_HUD_STATUS v3 bytes 5-6: the control request -----------------------
#
# SP-PROTOCOL-V3 section 1.2. Bytes 0-4 and 7 are byte-identical to v2 (section 7 above
# still checks them), so everything here is the two new bytes. The bit positions are
# transcribed from the board's own parser, sp_hud.c sp_hud_rx():
#   b5.0 WANT_CONTROL  b5.1 LAT_READY  b5.4:2 OP_STATE
#   b5.5 RELEASE_BRAKE b5.6 RELEASE_DRIVER  b5.7 LDW_ACTIVE   b6 MAX_TORQUE

print("\n[10] SP_HUD_STATUS v3 control request")
cc10, CP10, _, _ = build()


def v3_frame(lat_active=False, enabled=True, brake=False, steering_pressed=False,
             ldw_left=False, ldw_right=False, v_ego=25.0, torque=0.0, frames=30, start=0):
  """Run a few frames and return the last 0x500 payload plus the last 0x0E4 payload."""
  cs = CS()
  cs.out.vEgo, cs.out.aEgo = v_ego, 0.0
  cs.out.brakePressed = brake
  cs.out.steeringPressed = steering_pressed
  sp = steer = None
  for j in range(start, start + frames):
    cc = structs.CarControl.new_message()
    cc.enabled = enabled
    cc.longActive = False
    cc.latActive = lat_active
    cc.actuators.torque = torque
    cc.hudControl.speedVisible = True
    cc.hudControl.setSpeed = 30.0
    cc.hudControl.leftLaneDepart = ldw_left
    cc.hudControl.rightLaneDepart = ldw_right
    _, sends = cc10.update(cc.as_reader(), CC_SP, cs, j * int(1e7))
    for m in sends:
      a, d = as_tuple(m)[0], bytes(as_tuple(m)[1])
      if a == 0x500:
        sp = d
      if a == 0xE4:
        steer = d
  return sp, steer


OFF, READY, REQUESTING, ACTIVE, WITHDRAWING, FAULTED = 0, 1, 2, 3, 4, 5


def b5(d):
  return d[5]


def op_state(d):
  return (d[5] >> 2) & 0x07


f, _ = v3_frame(lat_active=False, enabled=False, v_ego=0.0, start=0)
check("v3: lateral off and not enabled -> OP_STATE off, WANT_CONTROL clear",
      f is not None and op_state(f) == OFF and not (b5(f) & 0x01), f"b5={f and f[5]:#04x}")
check("v3: MAX_TORQUE is 0 -- 'use your own authority', so the ladder lives in one place",
      f is not None and f[6] == 0, f"{f and f[6]}")

f, _ = v3_frame(lat_active=False, enabled=True, v_ego=25.0, start=100)
check("v3: enabled, lateral available, not asking -> READY and LAT_READY",
      f is not None and op_state(f) == READY and (b5(f) & 0x02), f"b5={f and f[5]:#04x}")

f, _ = v3_frame(lat_active=True, torque=0.0, v_ego=25.0, start=200)
check("v3: asking with a zero command -> REQUESTING, WANT_CONTROL set",
      f is not None and op_state(f) == REQUESTING and (b5(f) & 0x01), f"b5={f and f[5]:#04x}")

f, _ = v3_frame(lat_active=True, torque=0.5, v_ego=25.0, frames=80, start=300)
check("v3: asking with a non-zero command -> ACTIVE",
      f is not None and op_state(f) == ACTIVE, f"b5={f and f[5]:#04x}")

f, _ = v3_frame(lat_active=True, torque=0.5, brake=True, v_ego=25.0, frames=80, start=400)
check("v3: brake while asking -> WITHDRAWING and RELEASE_BRAKE",
      f is not None and op_state(f) == WITHDRAWING and (b5(f) & 0x20), f"b5={f and f[5]:#04x}")

f, _ = v3_frame(lat_active=False, enabled=True, brake=True, v_ego=25.0, start=500)
check("v3: braking with lateral OFF does not claim to be withdrawing",
      f is not None and op_state(f) != WITHDRAWING and not (b5(f) & 0x20), f"b5={f and f[5]:#04x}")

f, _ = v3_frame(lat_active=True, torque=0.5, steering_pressed=True, v_ego=25.0, frames=80, start=600)
check("v3: driver on the wheel while asking -> WITHDRAWING and RELEASE_DRIVER",
      f is not None and op_state(f) == WITHDRAWING and (b5(f) & 0x40), f"b5={f and f[5]:#04x}")

f, _ = v3_frame(lat_active=True, ldw_left=True, v_ego=25.0, start=700)
check("v3: LDW_ACTIVE follows a lane departure", f is not None and (b5(f) & 0x80), f"b5={f and f[5]:#04x}")
f, _ = v3_frame(lat_active=True, v_ego=25.0, start=800)
check("v3: LDW_ACTIVE clear with no departure", f is not None and not (b5(f) & 0x80), f"b5={f and f[5]:#04x}")


# --- 11. LDW into STEERING_CONTROL byte 2, and the bits that must stay zero ------
#
# 0x0E4 byte 2 bits 5:4 are SPECIFIED (SP-PROTOCOL-V3 section 4) to reach the camera's serial
# byte 2 bits 5:4, and are INERT on firmware 875ba124: the board's 0x0E4 parse reads only bits
# 7 and 2 (gw_active.c:759-773) and lkas_uart.c:449 hard-zeroes serial byte 2 bits 5:4. What
# is checked below is therefore openpilot's TRANSMISSION only -- it proves nothing about what
# the EPS or the cluster sees, and must not be read as proof the warning was delivered.
# Byte 2 bit 2 is the board's SERIAL_DOMAIN declaration: setting it while openpilot is still
# in the 2560 domain tells the board to take STEER_TORQUE as serial counts at unity gain and
# pins it at full authority from the first frame. It must be zero.

print("\n[11] LDW bits in STEERING_CONTROL byte 2")
_, st = v3_frame(lat_active=True, ldw_left=True, ldw_right=False, v_ego=25.0, start=900)
check("LDW_LEFT sets byte 2 bit 4 only", st is not None and (st[2] & 0x30) == 0x10, f"b2={st and st[2]:#04x}")
_, st = v3_frame(lat_active=True, ldw_left=False, ldw_right=True, v_ego=25.0, start=1000)
check("LDW_RIGHT sets byte 2 bit 5 only", st is not None and (st[2] & 0x30) == 0x20, f"b2={st and st[2]:#04x}")
_, st = v3_frame(lat_active=True, ldw_left=True, ldw_right=True, v_ego=25.0, start=1100)
check("both departures set both bits", st is not None and (st[2] & 0x30) == 0x30, f"b2={st and st[2]:#04x}")
check("SERIAL_DOMAIN (bit 2) is CLEAR -- openpilot is still in the 2560 domain",
      st is not None and not (st[2] & 0x04), f"b2={st and st[2]:#04x}")
check("bits 6, 3, 1 and 0 of byte 2 are zero", st is not None and not (st[2] & 0x4B), f"b2={st and st[2]:#04x}")
check("STEER_TORQUE_REQUEST (bit 7) still carries latActive", st is not None and (st[2] & 0x80), f"b2={st and st[2]:#04x}")
_, st = v3_frame(lat_active=False, enabled=True, ldw_left=True, v_ego=25.0, start=1200)
check("LDW is sent even with lateral off -- it is a warning, not a request",
      st is not None and (st[2] & 0x10) and not (st[2] & 0x80), f"b2={st and st[2]:#04x}")


# --- 12. Release on brake within 200 ms ---------------------------------------
#
# The stock camera drops LKAS_ON within about 20 frames of a brake press (c8/c9). This is
# imitation of stock, not fault avoidance. Two properties are load-bearing: it may only ever
# REDUCE the command, and it cannot latch.

print("\n[12] brake release ramp")
cc12, CP12, _, _ = build()
cs12 = CS()
cs12.out.vEgo, cs12.out.aEgo = 25.0, 0.0


def steer_torque(cc_obj, cs, j, lat_active=True, torque=-1.0):
  cc = structs.CarControl.new_message()
  cc.enabled = True
  cc.longActive = False
  cc.latActive = lat_active
  cc.actuators.torque = torque
  cc.hudControl.speedVisible = True
  cc.hudControl.setSpeed = 30.0
  _, sends = cc_obj.update(cc.as_reader(), CC_SP, cs, j * int(1e7))
  for m in sends:
    a, d = as_tuple(m)[0], bytes(as_tuple(m)[1])
    if a == 0xE4:
      return int.from_bytes(d[0:2], "big", signed=True)
  return None


railed = None
for j in range(300):                       # wind the rate limiter up to the rail
  railed = steer_torque(cc12, cs12, j)
check("a steady request reaches full scale with the brake up", railed is not None and abs(railed) > 2000, f"{railed}")

cs12.out.brakePressed = True
ramp = [steer_torque(cc12, cs12, 300 + j) for j in range(25)]
steps = [abs(a - b) for a, b in zip(ramp, ramp[1:], strict=False)]
check("brake: the command shrinks every frame", all(abs(b) <= abs(a) + 1 for a, b in zip(ramp, ramp[1:], strict=False)),
      f"{ramp[:6]}")
check("brake: zero within 20 frames (0.20 s)", ramp[19] == 0, f"frame 20 = {ramp[19]}, ramp={ramp[:21]}")
# The ceiling walks down 1/BRAKE_RELEASE_FRAMES of full scale per frame, so the withdrawal is
# an exact linear ramp: 2560/20 = 128 CAN counts, which the board scales by authority/2560 to
# 4 serial counts at authority 80 -- under the stock camera's p99 of 5 and the 10 that
# SP-PROTOCOL-V3 section 3 allows, and far under the 16 the stock camera has ever stepped.
check("brake: the withdrawal is a linear ramp of 128 CAN counts per frame (+-1 for rounding)",
      max(steps) <= 2560 // BRAKE_RELEASE_FRAMES + 1, f"max step {max(steps)}, steps={steps[:6]}")
check("brake: it stays at zero while the brake is held", steer_torque(cc12, cs12, 400) == 0)

cs12.out.brakePressed = False
back = [steer_torque(cc12, cs12, 500 + j) for j in range(300)]
check("brake released: it does NOT latch -- the command comes back", abs(back[-1]) > 2000, f"{back[-1]}")
check("brake released: the recovery is rate limited, not a step", abs(back[0]) < 200, f"first frame {back[0]}")

# the scale is a pure function of the frame count, so the monotonicity claim can be checked
# directly rather than only through the controller
scales, n = [], 0
for _ in range(BRAKE_RELEASE_FRAMES + 5):
  sc, n = brake_release_scale(True, n)
  scales.append(sc)
check("scale is monotonically non-increasing and never above 1 or below 0",
      all(0.0 <= x <= 1.0 for x in scales) and all(b <= a for a, b in zip(scales, scales[1:], strict=False)))
check("scale reaches exactly 0 at BRAKE_RELEASE_FRAMES", scales[BRAKE_RELEASE_FRAMES - 1] == 0.0, f"{scales}")
sc, n = brake_release_scale(False, n)
check("one frame with the brake up clears it completely", sc == 1.0 and n == 0, f"{sc} {n}")


# --- 13. GW_STEER_GRANT (0x70B) -> CarStateSP.linbusGateway ---------------------
#
# Absence is never permission. The board only began sending this in firmware 75aa91ee, so an
# older image, or one dropped frame too many, must read as NOT granted -- and must not cost
# openpilot its CAN, or it would refuse to engage for want of a frame that is allowed to be
# missing.

print("\n[13] GW_STEER_GRANT -> CarStateSP.linbusGateway")
CP13 = CarInterface.get_non_essential_params(PLATFORM)
CP13_SP = CarInterface.get_non_essential_params_sp(CP13, PLATFORM)
CI13 = CarInterface(CP13, CP13_SP)
packer13 = CANPacker(DBC[PLATFORM][Bus.pt])

IDLE, GRANT_READY, REQUESTED, INTRO, GRANT_ACTIVE, LIMITED, REFUSED, BOARD_FAULT = range(8)


def grant_step(i, values=None):
  frames = []
  if values is not None:
    frames.append(as_tuple(packer13.make_can_msg("GW_STEER_GRANT", 0, values)))
  _, cs_sp = CI13.update([((i + 1) * int(1e7), frames)])
  return cs_sp.linbusGateway


g = grant_step(0, None)
check("no 0x70B has ever arrived -> not valid, NOT granted", not g.grantValid and not g.granted)

g = grant_step(1, {"STATE": GRANT_ACTIVE, "REASON": 0, "AUTHORITY": 80, "EPS_ACK": 1,
                   "EPS_FRESH": 1, "CAM_LKAS_ON": 1, "APPLIED": 40, "MOTOR_TORQUE": -16,
                   "RETRY_IN": 0, "GRANT_COUNTER": 7})
check("a frame arrives -> valid and granted", g.grantValid and g.granted)
check("STATE, REASON and AUTHORITY decode", g.grantState == GRANT_ACTIVE and g.grantReason == 0 and g.authority == 80,
      f"{g.grantState} {g.grantReason} {g.authority}")
check("the EPS bits decode", g.epsAck and g.epsFresh and g.camLkasOn and not g.epsLatched)
check("APPLIED is signed with scale 2", g.applied == 40, f"{g.applied}")
check("MOTOR_TORQUE is signed with scale 4", g.motorTorque == -16, f"{g.motorTorque}")
check("not latched", not g.latchedUntilKeyOff)

g = grant_step(2, {"STATE": REFUSED, "REASON": 8, "EPS_LATCHED": 1, "EPS_ERROR_STATE": 4,
                   "RETRY_IN": 255, "GRANT_COUNTER": 8})
check("REFUSED is not granted", g.grantValid and not g.granted and g.grantState == REFUSED)
check("REASON reaches carStateSP for the driver-facing layer", g.grantReason == 8, f"{g.grantReason}")
check("EPS_ERROR_STATE 4 decodes", g.epsErrorState == 4, f"{g.epsErrorState}")
check("RETRY_IN 255 -> latchedUntilKeyOff", g.latchedUntilKeyOff and g.retryIn == 255, f"{g.retryIn}")

# EPS_LATCHED IS NOT A LATCH. Byte 3 bit 1 is the board's `refusing` flag (gw_active.c:1391),
# a timed hold -- 3 s (GW_NOACK_HOLD_MS) when the EPS simply has not acknowledged, which this
# EPS does routinely below about 60 km/h. RETRY_IN counts that hold down in seconds, and 255
# is the only thing the board ever says that means "not this key cycle" (gw_active.c:1401-06).
# Telling the driver to cycle the ignition over a 3 s hold is the failure this guards.
g = grant_step(3, {"STATE": REFUSED, "REASON": 9, "EPS_LATCHED": 1, "EPS_ERROR_STATE": 0,
                   "RETRY_IN": 2, "GRANT_COUNTER": 9})
check("a no-ack refusal reports the hold, not a key-cycle latch",
      g.epsLatched and g.retryIn == 2 and not g.latchedUntilKeyOff, f"retryIn={g.retryIn}")
check("REASON 9 (EPS not acknowledging) reaches carStateSP", g.grantReason == 9, f"{g.grantReason}")

g = grant_step(4, {"STATE": INTRO, "REASON": 15, "RETRY_IN": 0, "GRANT_COUNTER": 10})
check("INTRO counts as granted (the board is about to put torque on the wire)", g.granted)
check("the latch is NOT sticky on our side -- the board clearing it clears this",
      not g.latchedUntilKeyOff and g.retryIn == 0)

g = grant_step(5, {"STATE": LIMITED, "REASON": 15, "GRANT_COUNTER": 11})
check("LIMITED counts as granted", g.granted)
g = grant_step(6, {"STATE": REQUESTED, "REASON": 3, "GRANT_COUNTER": 12})
check("REQUESTED does not", not g.granted)

for i in range(7, 7 + 49):
  g = grant_step(i, None)
check("still valid one frame inside the 500 ms window", g.grantValid)
g = grant_step(57, None)
check("stale after 50 frames -> not valid and NOT granted", not g.grantValid and not g.granted)
check("a stale frame reports nothing rather than the last thing it heard",
      g.grantState == 0 and g.grantReason == 0 and g.authority == 0 and not g.epsAck)

st13 = CI13.can_parsers[Bus.pt].message_states[0x70B]
check("GW_STEER_GRANT is registered before the first update", 0x70B in CI13.can_parsers[Bus.pt].addresses)
check("GW_STEER_GRANT is liveness-exempt (ignore_alive)", st13.ignore_alive)
fresh13 = CarInterface(CP13, CP13_SP)
fresh13.update([(int(1e7), [])])
fresh_st13 = fresh13.can_parsers[Bus.pt].message_states[0x70B]
check("a board that never sends 0x70B still reads as a valid message, not a CAN timeout",
      fresh_st13.valid(int(5e9), False) and not fresh_st13.timestamps)
# THE trap this whole registration exists for: a message reached lazily through cp.vl[...]
# is registered with freq=None, takes a timeout threshold, and a message past its timeout
# makes can_valid false -- which feeds canValid and makes openpilot refuse to engage. The
# board's telemetry has been observed blacked out for 94.5 s. Drive it past the threshold
# and check the message still reports valid.
# (can_valid itself cannot be asserted here: this CarInterface has only ever been fed 0x70B,
# so every other car message on the bus is legitimately timed out.)
_cp13 = CI13.can_parsers[Bus.pt]
for i in range(57, 57 + 12000):            # 120 s of silence at the 100 Hz carstate rate
  grant_step(i, None)
_age_ns = _cp13._last_update_nanos - st13.timestamps[-1]
check("blacked out for far longer than its own timeout threshold",
      _age_ns > st13.timeout_threshold, f"{_age_ns / 1e9:.1f} s dark, threshold {st13.timeout_threshold / 1e9:.1f} s")
check("and it STILL reports valid, so it can never be the reason can_valid goes false",
      st13.valid(_cp13._last_update_nanos, False))

sigs13 = DBCFile(DBC[PLATFORM][Bus.pt]).addr_to_msg[0x70B].sigs
check("its counter is NOT named COUNTER (a honda_ DBC would enforce continuity and drop it)",
      "COUNTER" not in sigs13 and "GRANT_COUNTER" in sigs13, f"{sorted(sigs13)}")
check("it has no CHECKSUM signal (the board computes no Honda checksum for it)",
      "CHECKSUM" not in sigs13)

# --- 14. A latched STEER_TORQUE_SENSOR must not be read as driver intent ---------
#
# While the EPS is under LKAS control it stops updating STEER_TORQUE_SENSOR on 0x18F. The
# frame keeps arriving at 100 Hz with a rolling counter and a good checksum, so nothing in
# the CAN layer notices; carState.steeringTorque simply holds the value it had when the
# gateway engaged. Routes dd/de/df: frozen for up to 946 s at a time while the wheel moved,
# 41-69 % of each drive, canValid 1.00 throughout.
#
# Two failure modes came out of that, depending on what it latched at:
#   |latched| > 1200  -> steeringPressed stuck True  (integrator frozen for the whole
#                        engagement, driver monitoring told the driver is holding the wheel,
#                        torqued and lagd starved, "take over" alert silenced)
#   |latched| < 1200  -> steeringPressed stuck False (openpilot cannot see the driver fight
#                        it at all -- route dd, 946 s latched at -528)

print("\n[14] latched STEER_TORQUE_SENSOR -> driverTorqueStale")
CP10 = CarInterface.get_non_essential_params(PLATFORM)
CP10_SP = CarInterface.get_non_essential_params_sp(CP10, PLATFORM)
CI10 = CarInterface(CP10, CP10_SP)
packer10 = CANPacker(DBC[PLATFORM][Bus.pt])


def steer_step(i, torque, control_active, board=None, board_ok=True):
  """One 100 Hz frame of STEER_STATUS, optionally with the board's EPS mirror beside it."""
  msgs = [packer10.make_can_msg("STEER_STATUS", 0, {"STEER_TORQUE_SENSOR": torque,
                                                    "STEER_CONTROL_ACTIVE": control_active})]
  if board is not None:
    msgs.append(packer10.make_can_msg("EPS_LIN_RAW", 0, {"STEER_TORQUE": board,
                                                         "CHECKSUM_OK": board_ok}))
  frames = [m if isinstance(m, tuple) else (m.address, bytes(m.dat), m.src) for m in msgs]
  return CI10.update([((i + 1) * int(1e7), frames)])


_frame10 = [0]


def hold(torque, control_active, frames=120, moving=False, board=None, board_ok=True):
  """Feed `frames` of STEER_STATUS and return the final (CarState, CarStateSP)."""
  out = None
  for _ in range(frames):
    _frame10[0] += 1
    value = torque + (_frame10[0] % 7) * 13 if moving else torque
    b = None if board is None else board + (_frame10[0] % 5)
    out = steer_step(_frame10[0], value, control_active, b, board_ok)
  return out


# warm-up: lazy VLDict registration drops the first frame of any message (see section 9)
hold(0, 0, frames=1, board=0)

# a real driver holding a constant torque, NOT under LKAS control: never stale
cs10, cs_sp10 = hold(2000, 0)
check("constant torque with STEER_CONTROL_ACTIVE=0 is NOT stale", not cs_sp10.driverTorqueStale)
check("...and steeringPressed is left alone", cs10.steeringPressed)

# LKAS takes over and the value latches, with NO board frame to stand in for it
cs10, cs_sp10 = hold(2000, 1)
check("latched torque and no board frame IS stale", cs_sp10.driverTorqueStale)
check("steeringPressed is forced False rather than stuck True", not cs10.steeringPressed,
      "stuck True is what froze the integrator for 946 s on route dd")

# the EPS keeps reporting properly while under LKAS control (a fixed car): not stale
cs10, cs_sp10 = hold(2000, 1, moving=True)
check("a LIVE value under LKAS control is NOT stale", not cs_sp10.driverTorqueStale,
      "STEER_CONTROL_ACTIVE alone must not discard good data")
check("...and steeringPressed works again", cs10.steeringPressed)

# it must go stale again, and recover again, without a restart
cs10, cs_sp10 = hold(777, 1)
check("re-latching is detected again", cs_sp10.driverTorqueStale)

# the sub-threshold latch from route dd: -528 for 946 s, steeringPressed stuck False
cs10, cs_sp10 = hold(-528, 1)
check("the route-dd latch (-528, under the 1200 threshold) is reported stale",
      cs_sp10.driverTorqueStale,
      "it reads as 'no driver' either way, but only the flag says openpilot is BLIND")


# --- 14b. the board's EPS mirror stands in for the latched value -----------------
#
# EPS_LIN_RAW (0x700) is on the bus at 100 Hz on every live image, it stays live through the
# freeze (43.5 % of samples change while 0x18F is latched, route dd) and its CHECKSUM_OK was
# 1.000 on every frame of dd/de/df. It is the only live driver-torque signal on this car
# while openpilot is steering.

# latched at -528, board reporting a hard LEFT pull: -40 serial counts -> +2580 CAN counts
cs10, cs_sp10 = hold(-528, 1, board=-40)
check("a latched value with a live board frame is NOT stale", not cs_sp10.driverTorqueStale)
check("steeringTorque comes from the board, sign flipped to left-positive",
      cs10.steeringTorque > 2000, f"{cs10.steeringTorque:.0f}")
check("...and steeringPressed follows it", cs10.steeringPressed)

# the same latched -528, board reporting the driver is off the wheel
cs10, cs_sp10 = hold(-528, 1, board=0)
check("board says no driver torque -> steeringPressed False", not cs10.steeringPressed,
      f"{cs10.steeringTorque:.0f}")
check("...and still not stale, because the signal is good", not cs_sp10.driverTorqueStale)

# sign both ways: a RIGHT pull on the board must read negative in CarState
cs10, _ = hold(-528, 1, board=+40)
check("board positive -> CarState negative (right)", cs10.steeringTorque < -2000,
      f"{cs10.steeringTorque:.0f}")

# a bad EPS serial checksum must not be substituted
cs10, cs_sp10 = hold(-528, 1, board=-40, board_ok=False)
check("CHECKSUM_OK=0 is not substituted", cs_sp10.driverTorqueStale)
check("...and steeringPressed falls back to False", not cs10.steeringPressed)

# the board goes quiet mid-engagement: must return to stale, not hold its last word
cs10, cs_sp10 = hold(-528, 1, board=-40)
check("substituting again once the board speaks", not cs_sp10.driverTorqueStale)
cs10, cs_sp10 = hold(-528, 1)
check("board silent -> stale again, NOT the last torque it sent", cs_sp10.driverTorqueStale)
check("...and steeringPressed is False, not the stale substitute", not cs10.steeringPressed)

# while the car's own sensor is live the board is ignored, whatever it says
cs10, cs_sp10 = hold(2000, 0, board=-40)
check("a live car sensor wins over the board mirror", cs10.steeringTorque == 2000,
      f"{cs10.steeringTorque:.0f}")


# --- 15. A stale reading cannot confirm a lane change ----------------------------
#
# 14 of 17 lane changes on dd/de/df were confirmed within a single 0.05 s sample, and all
# 10 failures were the direction the latched sign happened to oppose. With the torque
# latched at +1813 every LEFT change fired instantly and every RIGHT one was impossible.

print("\n[15] lane change nudge ignores a latched torque")
try:
  import types

  from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState
except Exception as e:  # openpilot not importable standalone
  print(f"  SKIP  desire_helper not importable here ({type(e).__name__})")
else:
  def carstate(torque, pressed, blinker_left):
    return types.SimpleNamespace(vEgo=25.0, steeringTorque=torque, steeringPressed=pressed,
                                 leftBlinker=blinker_left, rightBlinker=not blinker_left,
                                 leftBlindspot=False, rightBlindspot=False, brakePressed=False)

  def run(stale, torque, blinker_left):
    dh = DesireHelper()
    dh.update(carstate(torque, True, blinker_left), True, 0.0, stale)      # blinker rising edge
    dh.update(carstate(torque, True, blinker_left), True, 0.0, stale)
    return dh.lane_change_state

  st = run(False, 3000, True)
  check("a LIVE matching nudge still confirms", st == LaneChangeState.laneChangeStarting, f"{st}")
  st = run(True, 3000, True)
  check("a STALE matching nudge does NOT confirm", st == LaneChangeState.preLaneChange, f"{st}")
  st = run(True, -3000, True)
  check("a STALE opposing value does not confirm either", st == LaneChangeState.preLaneChange, f"{st}")


print("\n" + "=" * 60)
if FAILURES:
  print(f"{len(FAILURES)} FAILED: {FAILURES}")
  sys.exit(1)
print("ALL CHECKS PASSED")
