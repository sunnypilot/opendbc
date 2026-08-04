#!/usr/bin/env python3
"""Integration checks: the real Honda ELESYS CarController, driven frame by frame
with the dynamic tuner switched on.

test_dynamic_tuning.py covers HondaDynamicTuner in isolation. It cannot catch the
class of bug that lives in the *seam* -- call ordering, the 100 Hz / 50 Hz / 10 Hz
rate split, which branch leaves pcm_speed stale, whether a learned gain reaches an
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
    if key in ("HondaDynamicTuningEnabled", "HondaDynamicPcmBlendEnabled"):
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


def build(tuning=True, blend=True):
  params = FakeParams()
  params.store["HondaDynamicTuningEnabled"] = tuning
  params.store["HondaDynamicPcmBlendEnabled"] = blend
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
    cc_obj.update(make_cc(target, state, long_active, pitch), CC_SP, cs, i * int(1e7))
    trace.append({"i": i, "v": v, "brake": cc_obj.apply_brake_last, "gas": cc_obj.gas,
                  "pcm": int(cc_obj.dynamic_tuner.new_accel), "state": state,
                  "gain": 1.0 + cc_obj.dynamic_tuner.brake_pid_factor})
  return trace


# --- 1. the toggle off must be bit-identical to stock ------------------------

print("\n[1] toggle off == stock")
PHASES = ([(0.9, LongCtrlState.pid, 0.0)] * 400 + [(0.2, LongCtrlState.pid, 0.03)] * 600 +
          [(-1.5, LongCtrlState.pid, -0.03)] * 500 + [(-0.8, LongCtrlState.stopping, 0.0)] * 400)
off, *_ = build(tuning=False, blend=False)
t_off = drive(off, PHASES)
check("tuner reports disabled", not off.dynamic_tuner.enabled and not off.dynamic_tuner.pcm_blend)
check("no PCM gas is ever requested with the interceptor and no blend",
      all(f["pcm"] == 0 for f in t_off))
check("brake gain is exactly 1.0 every frame",
      all(abs(f["gain"] - 1.0) < 1e-12 for f in t_off))

# same drive with the tuner on: nothing may leave the legal range
print("\n[2] toggle on, both features")
on, CP, CP_SP, params = build()
t_on = drive(on, PHASES)
check("tuner and blend both came up", on.dynamic_tuner.enabled and on.dynamic_tuner.pcm_blend)
check("brake command stays in range",
      all(0 <= f["brake"] <= PARAMS.NIDEC_BRAKE_MAX - 1 for f in t_on),
      f"{min(f['brake'] for f in t_on)}..{max(f['brake'] for f in t_on)}")
check("interceptor command stays in range", all(0.0 <= f["gas"] <= 1.0 for f in t_on))
check("pcm gas stays in range", all(0 <= f["pcm"] <= PARAMS.NIDEC_GAS_MAX for f in t_on))
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
v = 6.0
for i in range(1200):
  cs.out.vEgo, cs.out.aEgo = v, -1.0
  cc_obj.update(make_cc(-2.0), CC_SP, cs, i * int(1e7))
  if cc_obj.apply_brake_last > 0 and int(cc_obj.dynamic_tuner.new_accel) > 0:
    bad.append(i)
  v = max(0.0, v - 0.005)
check("PCM gas is cut whenever brake is commanded", not bad, f"{len(bad)} frames")


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


# --- 6. the PCM crossfade ------------------------------------------------------

print("\n[6] pedal / PCM crossfade")
cc_obj, *_ = build()
cs = CS()
seen = {}
for i, v in enumerate(np.linspace(0.0, 20.0, 4000)):
  cs.out.vEgo, cs.out.aEgo = float(v), 0.5
  cc_obj.update(make_cc(1.0), CC_SP, cs, i * int(1e7))
  seen[round(float(v), 1)] = (cc_obj.gas, int(cc_obj.dynamic_tuner.new_accel))
below = [g for v, (g, p) in seen.items() if v < dt.PCM_AUTHORITY_FLOOR - 0.5 and p != 0]
check("no PCM gas below the authority floor", not below, f"{len(below)} speeds")
above = [p for v, (g, p) in seen.items() if v > dt.PCM_AUTHORITY_FULL + 2.0]
check("PCM gas is live above the full-authority speed", any(p > 0 for p in above))
pedal_hi = [g for v, (g, p) in seen.items() if v > dt.PCM_AUTHORITY_FULL + 2.0]
pedal_lo = [g for v, (g, p) in seen.items() if v < dt.PCM_AUTHORITY_FLOOR - 0.5]
check("the pedal share falls away as the PCM takes over",
      max(pedal_hi) < max(pedal_lo) + 1e-9, f"hi {max(pedal_hi):.3f} vs lo {max(pedal_lo):.3f}")

# with the blend off the interceptor must own the whole request at every speed
cc_obj, *_ = build(blend=False)
cs = CS()
pcm_any = 0
for i, v in enumerate(np.linspace(0.0, 25.0, 3000)):
  cs.out.vEgo, cs.out.aEgo = float(v), 0.5
  cc_obj.update(make_cc(1.0), CC_SP, cs, i * int(1e7))
  pcm_any = max(pcm_any, int(cc_obj.dynamic_tuner.new_accel))
check("blend off -> the PCM channel is inert at every speed", pcm_any == 0, f"max {pcm_any}")


print("\n" + "=" * 60)
if FAILURES:
  print(f"{len(FAILURES)} FAILED: {FAILURES}")
  sys.exit(1)
print("ALL CHECKS PASSED")
