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

from opendbc.car import structs
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.carcontroller import CarController
from opendbc.car.honda.hondacan import honda_checksum
from opendbc.car.honda.values import CAR, CarControllerParams
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
    check("on bus 2, 8 bytes", bus == 2 and len(dat) == 8, f"bus={bus} dlc={len(dat)}")
    bad = [d for _, d in sp_frames if (d[7] & 0x0F) != honda_checksum(0x500, None, bytearray(d))]
    check("every frame carries a valid Honda checksum", not bad, f"{len(bad)} bad")
    check("protocol version is 1", (dat[0] >> 4) & 0x0F == 1, f"{(dat[0] >> 4) & 0x0F}")
    # set speed is km/h on the wire regardless of cluster units: 30 m/s -> 108
    check("SET_SPEED is km/h, not the cluster's display units", dat[2] == 108, f"{dat[2]}")
    counters = [(d[7] >> 4) & 0x03 for _, d in sp_frames]
    check("COUNTER advances", len(set(counters)) > 1, f"{sorted(set(counters))}")


print("\n" + "=" * 60)
if FAILURES:
  print(f"{len(FAILURES)} FAILED: {FAILURES}")
  sys.exit(1)
print("ALL CHECKS PASSED")
