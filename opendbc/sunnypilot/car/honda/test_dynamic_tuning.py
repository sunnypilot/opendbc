#!/usr/bin/env python3
"""Offline checks for HondaDynamicTuner. Runs against a stubbed opendbc surface.

These do not prove the tuning is *good* -- only road data can do that. They prove
the things that have to hold before it is ever flashed:
  1. with the toggle off, everything the car controller consumes is stock,
  2. the learners move the right way and settle,
  3. the clamps hold under adversarial input,
  4. no learner can rail on actuator lag, latch, or persist a transient.

Several checks below are named for the specific failure mode they regression-test.
"""

import sys
from dataclasses import dataclass, field

import numpy as np

from opendbc.car import structs
from opendbc.sunnypilot.car.honda import dynamic_tuning as dt
from opendbc.sunnypilot.car.honda.dynamic_tuning import (
  HondaDynamicTuner, _bp_weights, PEDAL_GAIN_BP, PEDAL_GAIN_MIN, PEDAL_GAIN_MAX,
  PCM_AUTHORITY_FLOOR, PCM_AUTHORITY_FULL, SETTLE_FRAMES, BRAKE_POS_LIMIT,
  SPEED_LEAD_MAX, SPEED_FACTOR_MAX, WIND_FACTOR_MAX, WIND_FACTOR_MIN,
  PITCH_ACCEL_LIMIT, PITCH_STALE_FRAMES, PCM_RAMP_PER_TX, GAS_ALPHA_MAX,
)

LongCtrlState = structs.CarControl.Actuators.LongControlState

MAX_SETTLE_FRAMES = 1000   # generous cap for settle(); real waits are 135-177

NIDEC_BRAKE_MAX = 256
NIDEC_GAS_MAX = 198

FAILURES = []


def check(name, cond, detail=""):
  if cond:
    print(f"  PASS  {name}")
  else:
    print(f"  FAIL  {name}  {detail}")
    FAILURES.append(name)


@dataclass
class Actuators:
  accel: float = 0.0
  longControlState: object = LongCtrlState.pid


@dataclass
class CC:
  longActive: bool = True
  orientationNED: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
  actuators: Actuators = field(default_factory=Actuators)


@dataclass
class Out:
  vEgo: float = 10.0
  aEgo: float = 0.0
  gasPressed: bool = False
  brakePressed: bool = False
  stockAeb: bool = False


@dataclass
class CS:
  out: Out = field(default_factory=Out)


class FakeParams:
  """Stands in for openpilot Params, including the UnknownKeyName behaviour."""

  def __init__(self, values=None, known=None):
    self.values = dict(values or {})
    self.known = known
    self.written = {}

  def _check(self, key):
    if self.known is not None and key not in self.known:
      raise KeyError(f"UnknownKeyName: {key}")

  def get(self, key, return_default=False):
    self._check(key)
    return self.values.get(key)

  def get_bool(self, key):
    self._check(key)
    return bool(self.values.get(key, False))

  def put(self, key, value):
    self._check(key)
    self.written[key] = value


def make_tuner(enabled=True, pcm_blend=False, params=None):
  if params is not None:
    dt._open_params = lambda: params
  else:
    dt._open_params = lambda: None
  t = HondaDynamicTuner(CP=None, CP_SP=None)
  if params is None:
    t.enabled = enabled
    t.pcm_blend = enabled and pcm_blend
  return t


def settle(t, cc, cs, n=None):
  """Drive update_state until the dwell is actually open.

  This used to run a fixed SETTLE_FRAMES + 5 frames, which was enough when the
  dwell started counting from the first steady frame. It now counts only once the
  lag model has caught up with the commanded step, so the wait depends on the step
  size -- measured 135 frames for a 0.5 m/s^2 step, 156 for 1.0, 177 for 2.0.
  Waiting on the real condition keeps the tests honest about that instead of
  encoding a constant that has to be re-derived every time PLANT_TAU moves.
  """
  if n is not None:
    for _ in range(n):
      t.update_state(cc, cs)
    return
  for _ in range(MAX_SETTLE_FRAMES):
    t.update_state(cc, cs)
    if t._settle >= SETTLE_FRAMES:
      return
  raise AssertionError(f"dwell never opened within {MAX_SETTLE_FRAMES} frames (settle={t._settle})")


def base(v=10.0, target=0.0, a=0.0):
  cc, cs = CC(), CS()
  cc.actuators.accel = target
  cs.out.vEgo, cs.out.aEgo = v, a
  return cc, cs


# --- 1. disabled is a strict no-op ------------------------------------------

print("\n[1] toggle off -> stock behaviour")
t = make_tuner(enabled=False)
cc, cs = base(12.0, 1.0)
cc.orientationNED = [0.0, 0.30, 0.0]
check("pitch feedforward is exactly 0.0", t.update_state(cc, cs) == 0.0)
check("pedal gain is exactly 1.0", t.pedal_gain_at(12.0) == 1.0)
check("wind scale is exactly 1.0", t.wind_scale() == 1.0)
check("pcm authority is 0.0 at all speeds", all(t.pcm_authority(v) == 0.0 for v in (0, 5, 10, 20, 40)))
check("pedal authority is 1.0 at all speeds", all(t.pedal_authority(v) == 1.0 for v in (0, 5, 10, 20, 40)))
check("brake gain is exactly 1.0", t.brake_gain(cc, cs, 0.5) == 1.0)
check("pcm_request is silent", t.pcm_request(cc, cs, 1.0, 0.2, 2.0, NIDEC_GAS_MAX) == (0.0, 0))

before = list(t.pedal_gain)
for _ in range(500):
  t.update_state(cc, cs)
  t.update_pedal(cc, cs, 0.5)
  t.update_wind(cc, cs, 0.2)
check("no learner moves when disabled", t.pedal_gain == before and t.wind_factor == 1.0)

# gain of exactly 1.0 must be an identity on the command, not just close
gas = 0.4213
check("disabled gain is an exact identity on gas", gas * t.pedal_gain_at(12.0) == gas)
check("disabled wind scale is an exact identity", gas * t.wind_scale() == gas)


# --- 2. pitch feedforward ----------------------------------------------------

print("\n[2] pitch feedforward")
t = make_tuner()
cc, cs = base()
cc.orientationNED = [0.0, 0.10, 0.0]
for _ in range(500):
  up = t.update_state(cc, cs)
check("uphill adds positive demand", up > 0)
cc.orientationNED = [0.0, -0.10, 0.0]
for _ in range(500):
  down = t.update_state(cc, cs)
check("downhill subtracts demand", down < 0)

t = make_tuner()
cc, cs = base()
cc.orientationNED = [0.0, 1.4, 0.0]     # absurd 80-degree slope
for _ in range(2000):
  v = t.update_state(cc, cs)
check("pitch feedforward is clamped", abs(v) <= PITCH_ACCEL_LIMIT + 1e-9, f"{v}")

t = make_tuner()
cc, cs = base()
cc.orientationNED = [0.0, float("nan"), 0.0]
vals = [t.update_state(cc, cs) for _ in range(200)]
check("NaN pitch never reaches the output", all(np.isfinite(x) for x in vals))
cc.orientationNED = [0.0, float("inf"), 0.0]
vals = [t.update_state(cc, cs) for _ in range(200)]
check("inf pitch never reaches the output", all(np.isfinite(x) for x in vals))

# stale pose must ramp out, not freeze forever  (review finding 8)
t = make_tuner()
cc, cs = base()
cc.orientationNED = [0.0, 0.15, 0.0]
for _ in range(1000):
  held = t.update_state(cc, cs)
cc.orientationNED = []                    # pose lost
for _ in range(PITCH_STALE_FRAMES + 2000):
  after = t.update_state(cc, cs)
check("stale pose ramps the feedforward out", abs(after) < 0.05 * abs(held), f"held={held:.3f} after={after:.3f}")


# --- 3. breakpoint weights ---------------------------------------------------

print("\n[3] breakpoint weighting")
check("below first bp -> all weight on bp0", _bp_weights(-1.0, PEDAL_GAIN_BP) == [(0, 1.0)])
check("above last bp -> all weight on last", _bp_weights(99.0, PEDAL_GAIN_BP) == [(5, 1.0)])
w = _bp_weights(4.5, PEDAL_GAIN_BP)
check("midpoint splits evenly", abs(w[0][1] - 0.5) < 1e-9 and abs(w[1][1] - 0.5) < 1e-9, str(w))
check("weights sum to 1 everywhere",
      all(abs(sum(x[1] for x in _bp_weights(float(v), PEDAL_GAIN_BP)) - 1.0) < 1e-9
          for v in np.linspace(0, 25, 200)))


# --- 4. pedal learner --------------------------------------------------------

print("\n[4] pedal learner")
SHORTFALL = 0.65
t = make_tuner()
cc, cs = base(10.0, 1.0)
settle(t, cc, cs)
for _ in range(30000):
  t.update_state(cc, cs)
  cs.out.aEgo = cc.actuators.accel * SHORTFALL * t.pedal_gain_at(cs.out.vEgo)
  t.update_pedal(cc, cs, 0.5)
achieved = cc.actuators.accel * SHORTFALL * t.pedal_gain_at(10.0)
check("gain grew to close the shortfall", t.pedal_gain_at(10.0) > 1.3)
check("achieved accel converged on target", abs(achieved - 1.0) < 0.05, f"{achieved:.3f}")
check("converged estimate tracked the live value",
      abs(t.pedal_gain_converged[3] - t.pedal_gain[3]) < 0.05,
      f"live={t.pedal_gain[3]:.3f} conv={t.pedal_gain_converged[3]:.3f}")

t = make_tuner()
cc, cs = base(10.0, 1.0)
settle(t, cc, cs)
for _ in range(20000):
  t.update_state(cc, cs)
  cs.out.aEgo = cc.actuators.accel * 1.6 * t.pedal_gain_at(cs.out.vEgo)
  t.update_pedal(cc, cs, 0.5)
check("gain shrinks when over-delivering", t.pedal_gain_at(10.0) < 0.9)

# clamps
t = make_tuner()
cc, cs = base(10.0, 4.0, -4.0)
settle(t, cc, cs)
for _ in range(6000):
  t.update_state(cc, cs)
  t.update_pedal(cc, cs, 0.5)
check("gain clamps at PEDAL_GAIN_MAX", max(t.pedal_gain) <= PEDAL_GAIN_MAX + 1e-9)
check("railed value is NOT written to the converged estimate",
      max(t.pedal_gain_converged) < PEDAL_GAIN_MAX - 0.1,
      f"conv={max(t.pedal_gain_converged):.3f}")

t = make_tuner()
cc, cs = base(10.0, 4.0, 40.0)
settle(t, cc, cs)
for _ in range(6000):
  t.update_state(cc, cs)
  t.update_pedal(cc, cs, 0.5)
check("gain clamps at PEDAL_GAIN_MIN", min(t.pedal_gain) >= PEDAL_GAIN_MIN - 1e-9)

# gates
for label, mutate in [
  ("gas pressed", lambda c, s: setattr(s.out, "gasPressed", True)),
  ("brake pressed", lambda c, s: setattr(s.out, "brakePressed", True)),
  ("stock AEB", lambda c, s: setattr(s.out, "stockAeb", True)),
  ("long inactive", lambda c, s: setattr(c, "longActive", False)),
  ("not in PID state", lambda c, s: setattr(c.actuators, "longControlState", LongCtrlState.stopping)),
]:
  t = make_tuner()
  cc, cs = base(10.0, 2.0)
  settle(t, cc, cs)
  mutate(cc, cs)
  before = list(t.pedal_gain)
  for _ in range(2000):
    t.update_state(cc, cs)
    t.update_pedal(cc, cs, 0.5)
  check(f"no learning while {label}", t.pedal_gain == before)

for label, cmd in [("railed high", 1.0), ("zero", 0.0)]:
  t = make_tuner()
  cc, cs = base(10.0, 2.0)
  settle(t, cc, cs)
  before = list(t.pedal_gain)
  for _ in range(2000):
    t.update_state(cc, cs)
    t.update_pedal(cc, cs, cmd)
  check(f"no learning when gas command is {label}", t.pedal_gain == before)

t = make_tuner()
cc, cs = base(10.0)
before = list(t.pedal_gain)
for i in range(4000):
  cc.actuators.accel = 2.0 if (i // 3) % 2 == 0 else -1.0
  t.update_state(cc, cs)
  t.update_pedal(cc, cs, 0.5)
check("dwell guard blocks learning on a thrashing target", t.pedal_gain == before)


# --- 5. brake learner --------------------------------------------------------

print("\n[5] brake learner")
t = make_tuner()
cc, cs = base(10.0, -2.0, -1.0)
settle(t, cc, cs)
for _ in range(1500):
  t.update_state(cc, cs)
  g = t.brake_gain(cc, cs, 0.5)
check("brake gain rises when under-braking", g > 1.0)
check("brake gain respects pos_limit", g <= 1.0 + BRAKE_POS_LIMIT + 1e-6, f"{g}")

# REGRESSION (review finding 1): a normal deceleration must not rail the gain on
# pure hydraulic lag. Model the actuator as a first-order lag on aEgo.
t = make_tuner()
cc, cs = base(15.0, 0.0, 0.0)
tau_alpha = 0.02          # ~0.5 s time constant at 100 Hz
peak = 1.0
for i in range(6000):
  cc.actuators.accel = -1.5 if (i // 600) % 2 == 0 else 0.0   # repeated brake applications
  t.update_state(cc, cs)
  cs.out.aEgo += tau_alpha * (cc.actuators.accel - cs.out.aEgo)   # plant tracks perfectly, just late
  peak = max(peak, t.brake_gain(cc, cs, 0.35))
check("lag alone does not rail the brake gain", peak < 1.0 + BRAKE_POS_LIMIT - 0.05, f"peak={peak:.3f}")
check("lag alone does not poison the converged estimate", t.brake_gain_converged < 0.2,
      f"conv={t.brake_gain_converged:.3f}")

# REGRESSION: the worst case, expressed in brake counts on the wire
worst_frac = 0.35
worst_counts = int(np.clip(worst_frac * (1.0 + BRAKE_POS_LIMIT) * NIDEC_BRAKE_MAX, 0, NIDEC_BRAKE_MAX - 1))
stock_counts = int(worst_frac * NIDEC_BRAKE_MAX)
check("max learned gain cannot saturate the brake command",
      worst_counts < NIDEC_BRAKE_MAX - 1, f"{stock_counts} -> {worst_counts} of {NIDEC_BRAKE_MAX}")

# REGRESSION (review finding 1): a railed episode must not survive a stop
t = make_tuner()
cc, cs = base(15.0, -2.0, 0.5)     # huge sustained error, drives the integrator to the rail
settle(t, cc, cs)
for _ in range(3000):
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.9)
railed = t.brake_pid.i
cs.out.vEgo, cs.out.aEgo = 0.0, 0.0
t.update_state(cc, cs)
after_stop = t.brake_gain(cc, cs, 0.9)
check("railed integrator does not survive a standstill",
      after_stop < 1.0 + 0.1, f"railed={railed:.3f} after_stop={after_stop:.3f}")
check("railed value is never written to the converged estimate", t.brake_gain_converged < 0.1,
      f"conv={t.brake_gain_converged:.3f}")

# REGRESSION (review finding 3): the stopping phase must not apply a wound gain open-loop
t = make_tuner()
cc, cs = base(15.0, -2.0, 0.5)
settle(t, cc, cs)
for _ in range(3000):
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.9)
cc.actuators.longControlState = LongCtrlState.stopping
t.update_state(cc, cs)
g_stopping = t.brake_gain(cc, cs, 0.9)
check("stopping caps the gain at the converged value", g_stopping <= 1.0 + t.brake_gain_converged + 1e-9,
      f"{g_stopping:.3f}")

# over-braking. The gain is two-sided on purpose -- it exists to trim the /2.6
# divisor in compute_gb_honda_elesys() live, and a one-sided gain could only ever
# add brake -- but the floor is deliberately much tighter than the ceiling.
t = make_tuner()
cc, cs = base(10.0, -1.0, -3.0)
settle(t, cc, cs)
for _ in range(3000):
  t.update_state(cc, cs)
  g = t.brake_gain(cc, cs, 0.5)
check("sustained over-braking does reduce the gain", g < 1.0, f"{g:.4f}")
check("but never below the floor", g >= 1.0 - dt.BRAKE_NEG_LIMIT - 1e-9,
      f"{g:.4f} vs floor {1.0 - dt.BRAKE_NEG_LIMIT:.2f}")
check("the floor is much tighter than the ceiling (under-braking is the worse failure)",
      dt.BRAKE_NEG_LIMIT < BRAKE_POS_LIMIT / 2,
      f"neg {dt.BRAKE_NEG_LIMIT} vs pos {BRAKE_POS_LIMIT}")
check("a railed-low gain does not reach the converged estimate",
      t.brake_gain_converged > -dt.BRAKE_NEG_LIMIT + 1e-6, f"{t.brake_gain_converged:.5f}")
# and it comes back up when the car is under-braking again
cc, cs = base(10.0, -1.0, -0.2)
settle(t, cc, cs)
for _ in range(3000):
  t.update_state(cc, cs)
  g_up = t.brake_gain(cc, cs, 0.5)
check("and it recovers upward once the error flips", g_up > g, f"{g:.4f} -> {g_up:.4f}")

for label, mutate in [
  ("stock AEB", lambda c, s: setattr(s.out, "stockAeb", True)),
  ("brake pressed", lambda c, s: setattr(s.out, "brakePressed", True)),
  ("gas pressed", lambda c, s: setattr(s.out, "gasPressed", True)),
  ("not in PID state", lambda c, s: setattr(c.actuators, "longControlState", LongCtrlState.starting)),
  ("long inactive", lambda c, s: setattr(c, "longActive", False)),
]:
  t = make_tuner()
  cc, cs = base(10.0, -2.0, -1.0)
  settle(t, cc, cs)
  mutate(cc, cs)
  before = t.brake_pid.i
  for _ in range(1000):
    t.update_state(cc, cs)
    t.brake_gain(cc, cs, 0.5)
  check(f"brake integrator frozen while {label}", abs(t.brake_pid.i - before) < 1e-9)

t = make_tuner()
cc, cs = base(0.5, -2.0, -1.0)      # below BRAKE_LEARN_MIN_SPEED
settle(t, cc, cs)
before = t.brake_pid.i
for _ in range(2000):
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.5)
check("brake integrator frozen below the learn floor", abs(t.brake_pid.i - before) < 1e-9)

t = make_tuner()
cc, cs = base(10.0, -2.0, -1.0)
before = t.brake_pid.i
for _ in range(30):                 # dwell not yet satisfied
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.5)
check("brake integrator frozen before the dwell opens", abs(t.brake_pid.i - before) < 1e-9)


# --- 6. pcm crossfade --------------------------------------------------------

print("\n[6] pcm crossfade")
t = make_tuner(pcm_blend=True)
check("no pcm authority below the floor", t.pcm_authority(PCM_AUTHORITY_FLOOR - 0.1) == 0.0)
check("full pcm authority above the band", t.pcm_authority(PCM_AUTHORITY_FULL + 5) == 1.0)
mid = (PCM_AUTHORITY_FLOOR + PCM_AUTHORITY_FULL) / 2
check("half authority mid-band", abs(t.pcm_authority(mid) - 0.5) < 1e-6)
check("pedal + pcm authority always sums to 1",
      all(abs(t.pcm_authority(float(v)) + t.pedal_authority(float(v)) - 1.0) < 1e-9
          for v in np.linspace(0, 40, 300)))
check("crossfade off -> pcm authority 0 everywhere",
      all(make_tuner(pcm_blend=False).pcm_authority(v) == 0.0 for v in np.linspace(0, 40, 50)))

# REGRESSION (review finding 6): pcm_speed must ramp in with authority, not step
t = make_tuner(pcm_blend=True)
leads = []
for v in np.linspace(PCM_AUTHORITY_FLOOR - 1.0, PCM_AUTHORITY_FULL + 1.0, 400):
  cc, cs = base(float(v), 2.0)
  t.update_state(cc, cs)
  sp, _ = t.pcm_request(cc, cs, 2.0 * t.pcm_authority(float(v)), 0.2, 2.0, NIDEC_GAS_MAX)
  leads.append(0.0 if sp == 0.0 else sp - float(v))
steps = [abs(leads[i + 1] - leads[i]) for i in range(len(leads) - 1)]
check("pcm speed lead ramps in continuously", max(steps) < 0.25, f"max step={max(steps):.3f} m/s")
check("pcm speed lead is bounded", max(leads) <= SPEED_LEAD_MAX + 1e-9, f"{max(leads):.3f}")

t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 0.0)
t.update_state(cc, cs)
sp0, _ = t.pcm_request(cc, cs, 0.0, 0.2, 2.0, NIDEC_GAS_MAX)
check("pcm_request holds vEgo when target accel is zero", abs(sp0 - 25.0) < 1e-6, f"{sp0}")


# --- 7. pcm feedforward ------------------------------------------------------

print("\n[7] pcm feedforward")
# REGRESSION (review finding 11): the ramp limit must bound what the CAR sees
# (10 Hz), not what is recomputed internally (100 Hz).
t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 2.0)
t.update_state(cc, cs)
tx, prev_tx, ok = [], 0, True
for frame in range(600):
  _, pa = t.pcm_request(cc, cs, 2.0, 0.2, 2.0, NIDEC_GAS_MAX)
  if frame % 10 == 0:                       # the only frames the HUD goes out
    if pa > prev_tx + PCM_RAMP_PER_TX:
      ok = False
      break
    prev_tx = pa
    t.note_pcm_tx(pa)
    tx.append(pa)
check("ramp limit bounds change per TRANSMITTED frame", ok, f"jump to {prev_tx}")
check("feedforward never exceeds NIDEC_GAS_MAX", max(tx) <= NIDEC_GAS_MAX, f"{max(tx)}")

t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 2.0)
cc.longActive = False
check("feedforward resets when long is inactive",
      t.pcm_request(cc, cs, 2.0, 0.2, 2.0, NIDEC_GAS_MAX) == (0.0, 0) and t.prior_gas_average == 0.0)

t = make_tuner(pcm_blend=True)
t.new_accel, t.prior_gas_average, t.pcm_accel_raw, t.pcm_accel_last_tx = 50, 40.0, 60, 55
t.reset_pcm_feedforward()
check("reset_pcm_feedforward clears every piece of state",
      (t.new_accel, t.prior_gas_average, t.pcm_accel_raw, t.pcm_accel_last_tx) == (0.0, 0.0, 0, 0))


# --- 8. pcm learners ---------------------------------------------------------

print("\n[8] pcm learners")
# REGRESSION (review finding 5): gas_alpha must not be able to command gas on its own
t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 2.0, -2.0)
settle(t, cc, cs)
for _ in range(8000):
  t.update_state(cc, cs)
  t.pcm_request(cc, cs, 2.0, 0.2, 2.0, NIDEC_GAS_MAX)
  t.update_pcm(cc, cs, 2.0, NIDEC_GAS_MAX, False)
check("gas_alpha clamps", abs(t.gas_alpha) <= GAS_ALPHA_MAX + 1e-9, f"{t.gas_alpha}")
alpha_only = int(np.clip(GAS_ALPHA_MAX / 0.6, 0.0, 1.0) * NIDEC_GAS_MAX)
check("gas_alpha alone cannot saturate the channel", alpha_only < NIDEC_GAS_MAX // 2,
      f"alpha-only request = {alpha_only}/{NIDEC_GAS_MAX}")

# REGRESSION (review finding 4): speed_factor must not latch after a railed episode
t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 1.5, -0.5)
settle(t, cc, cs)
for _ in range(12000):                       # long railed episode
  t.update_state(cc, cs)
  t.new_accel = NIDEC_GAS_MAX
  t.update_pcm(cc, cs, 1.5, NIDEC_GAS_MAX, False)
latched = t.speed_factor
check("speed_factor is bounded while railed", latched <= SPEED_FACTOR_MAX + 1e-9, f"{latched}")
cs.out.aEgo = 1.5                            # now tracking perfectly, off the rail
for _ in range(12000):
  t.update_state(cc, cs)
  t.new_accel = 100
  t.update_pcm(cc, cs, 1.5, NIDEC_GAS_MAX, False)
check("speed_factor relaxes once off the rail", t.speed_factor < latched - 0.1,
      f"latched={latched:.3f} relaxed={t.speed_factor:.3f}")

t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 2.0, -2.0)
settle(t, cc, cs)
for _ in range(8000):
  t.update_state(cc, cs)
  t.new_accel = 100
  t.update_pcm(cc, cs, 2.0, NIDEC_GAS_MAX, False)
check("average_factor stays in range and non-zero", 0.30 - 1e-9 <= t.average_factor <= 1.0 + 1e-9,
      f"{t.average_factor}")
check("gas_factor stays in range", 0.5 - 1e-9 <= t.gas_factor <= 2.0 + 1e-9, f"{t.gas_factor}")

t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 2.0, -2.0)
settle(t, cc, cs)
before = (t.gas_factor, t.gas_alpha, t.speed_factor)
for _ in range(5000):
  t.update_state(cc, cs)
  t.update_pcm(cc, cs, 2.0, NIDEC_GAS_MAX, True)      # braking
check("no pcm learning while braking", (t.gas_factor, t.gas_alpha, t.speed_factor) == before)

t = make_tuner(pcm_blend=False)
cc, cs = base(25.0, 2.0, -2.0)
settle(t, cc, cs)
before = (t.gas_factor, t.gas_alpha, t.speed_factor)
for _ in range(5000):
  t.update_state(cc, cs)
  t.update_pcm(cc, cs, 2.0, NIDEC_GAS_MAX, False)
check("no pcm learning when the crossfade is off",
      (t.gas_factor, t.gas_alpha, t.speed_factor) == before)


# --- 9. wind learner ---------------------------------------------------------

print("\n[9] wind learner")
t = make_tuner()
cc, cs = base(30.0, 1.0, -3.0)
settle(t, cc, cs)
for _ in range(8000):
  t.update_state(cc, cs)
  t.update_wind(cc, cs, 0.441)
check("wind factor clamps high", t.wind_factor <= WIND_FACTOR_MAX + 1e-9, f"{t.wind_factor}")

t = make_tuner()
cc, cs = base(30.0, 1.0, 5.0)
settle(t, cc, cs)
for _ in range(8000):
  t.update_state(cc, cs)
  t.update_wind(cc, cs, 0.441)
check("wind factor clamps low", t.wind_factor >= WIND_FACTOR_MIN - 1e-9, f"{t.wind_factor}")

# REGRESSION (review finding 9): a zero-mean error must not ratchet the factor
t = make_tuner()
cc, cs = base(30.0, 1.0)
settle(t, cc, cs)
for i in range(40000):
  cs.out.aEgo = 1.0 - (0.5 if i % 2 == 0 else -0.5)     # symmetric error
  t.update_state(cc, cs)
  t.update_wind(cc, cs, 0.441)
check("zero-mean error does not drift the wind factor", abs(t.wind_factor - 1.0) < 0.02,
      f"{t.wind_factor:.5f}")


# --- 10. params: load clamping, unknown keys, persistence -------------------

print("\n[10] params handling")
known = set(dt._PARAM_SPEC) | {"HondaDynamicTuningEnabled", "HondaDynamicPcmBlendEnabled"}

# REGRESSION (review finding 14): corrupted / hand-edited values must be clamped
p = FakeParams({"HondaDynamicTuningEnabled": True, "HondaDynBrakeGain": 10.0,
                "HondaDynPedalGain0": 99.0, "HondaDynSpeedFactor": 500.0,
                "HondaDynGasAlpha": -50.0, "HondaDynAverageFactor": 0.0}, known)
t = make_tuner(params=p)
check("out-of-range brake gain is clamped on load", t.brake_pid.i <= BRAKE_POS_LIMIT + 1e-9,
      f"{t.brake_pid.i}")
check("out-of-range pedal gain is clamped on load", t.pedal_gain[0] <= PEDAL_GAIN_MAX + 1e-9)
check("out-of-range speed factor is clamped on load", t.speed_factor <= SPEED_FACTOR_MAX + 1e-9)
check("out-of-range gas alpha is clamped on load", t.gas_alpha >= -GAS_ALPHA_MAX - 1e-9)
check("zero average factor is clamped away from a divide-by-zero", t.average_factor >= 0.30 - 1e-9)
cc, cs = base(10.0, -2.0, -1.0)
check("clamped brake gain reaches the output bounded",
      t.brake_gain(cc, cs, 1.0) <= 1.0 + BRAKE_POS_LIMIT + 1e-9)

p = FakeParams({"HondaDynamicTuningEnabled": True, "HondaDynPedalGain2": float("nan"),
                "HondaDynWindFactor": float("inf")}, known)
t = make_tuner(params=p)
check("NaN param falls back to the default", np.isfinite(t.pedal_gain[2]))
check("inf param is clamped", np.isfinite(t.wind_factor) and t.wind_factor <= WIND_FACTOR_MAX + 1e-9)

# unknown keys (params_keys.h not updated) must degrade, not crash
p = FakeParams({}, known=set())
t = make_tuner(params=p)
check("unregistered keys degrade to disabled without raising", t.enabled is False)
check("no writer thread started when disabled", t._writer is None)

# persistence writes the CONVERGED values only
p = FakeParams({"HondaDynamicTuningEnabled": True}, known)
t = make_tuner(params=p)
t.pedal_gain = [1.7] * 6
t.pedal_gain_converged = [1.1] * 6
t.brake_pid_factor, t.brake_gain_converged = 0.6, 0.12
t.persist(dt.PERSIST_INTERVAL)
# the writer is a daemon thread with no task_done()/join() contract; poll instead
import time as _time
for _ in range(100):
  if "HondaDynBrakeGain" in p.written:
    break
  _time.sleep(0.01)
check("persist writes the converged pedal gain, not the live one",
      abs(p.written.get("HondaDynPedalGain0", -1) - 1.1) < 1e-9, str(p.written.get("HondaDynPedalGain0")))
check("persist writes the converged brake gain, not the live one",
      abs(p.written.get("HondaDynBrakeGain", -1) - 0.12) < 1e-9, str(p.written.get("HondaDynBrakeGain")))
t2 = make_tuner(params=FakeParams({"HondaDynamicTuningEnabled": True}, known))
t2.persist(dt.PERSIST_INTERVAL + 1)
check("persist is a no-op off the interval", len(t2._writer._queue.queue) == 0)


# --- 11. standalone / no-openpilot ------------------------------------------

print("\n[11] standalone import")
dt._open_params = lambda: None
t = HondaDynamicTuner(CP=None, CP_SP=None)   # no helper override: the real default path
check("constructs with no openpilot Params available", t is not None)
check("defaults to disabled without params", t.enabled is False)
check("no writer thread without params", t._writer is None)
check("persist() is a no-op without params", t.persist(dt.PERSIST_INTERVAL) is None)
check("debug_values() works", isinstance(t.debug_values(), dict))


# --- 12. second-review regressions ------------------------------------------

print("\n[12] second-review regressions")

# The original dwell gate compared frame-to-frame, i.e. a 20 m/s^3 jerk threshold,
# so it never closed on a realistic RAMPED target and the brake integrator ate
# pure hydraulic lag all the way to the rail. Step targets pass either way -- this
# uses the ramp shape a real planner produces.
t = make_tuner()
cc, cs = base(15.0, 0.0, 0.0)
peak, target = 1.0, 0.0
for i in range(60000):
  want = -2.0 if (i // 3000) % 2 == 0 else 0.0
  target += float(np.clip(want - target, -0.02, 0.02))     # 2 m/s^3 ramp
  cc.actuators.accel = target
  t.update_state(cc, cs)
  cs.out.aEgo += 0.02 * (target - cs.out.aEgo)             # perfect plant, 0.5 s lag only
  peak = max(peak, t.brake_gain(cc, cs, max(0.0, -target) * 0.2))
check("ramped target: lag alone does not rail the brake gain",
      peak < 1.0 + BRAKE_POS_LIMIT - 0.05, f"peak={peak:.4f}")
check("ramped target: lag alone does not reach disk",
      t.brake_gain_converged < 0.1, f"conv={t.brake_gain_converged:.4f}")

# A rolling stop enters and leaves `stopping` without ever reaching standstill.
# Clamping only the output left the integrator wound and handed it straight back.
t = make_tuner()
cc, cs = base(15.0, -2.0, 0.5)
settle(t, cc, cs)
for _ in range(3000):
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.9)
cc.actuators.longControlState = LongCtrlState.stopping
cs.out.vEgo = 1.5
for _ in range(200):
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.9)
cc.actuators.longControlState = LongCtrlState.pid
cs.out.vEgo = 2.0
t.update_state(cc, cs)
rolling = t.brake_gain(cc, cs, 0.9)
check("rolling stop does not hand back the wound gain", rolling <= 1.0 + 0.1, f"{rolling:.3f}")

# gas_alpha must be crossfaded like every other term, or a railed bias appears
# as real PCM gas the instant authority leaves zero.
t = make_tuner(pcm_blend=True)
t.gas_alpha = GAS_ALPHA_MAX
v_just_above = PCM_AUTHORITY_FLOOR + 0.02
cc, cs = base(v_just_above, 0.0)
t.update_state(cc, cs)
_, pa = t.pcm_request(cc, cs, 0.0, 0.0, 0.6, NIDEC_GAS_MAX)
check("railed gas_alpha contributes ~no gas at zero authority", pa <= 2, f"{pa} counts")

# average_factor was being learned against the ramp limiter rather than the PCM,
# which drove it to 1.0 and turned the feedforward into an identity.
t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 0.0, 0.0)
target = 0.0
for i in range(30000):
  want = 1.5 if (i // 2000) % 2 == 0 else 0.0
  target += float(np.clip(want - target, -0.02, 0.02))
  cc.actuators.accel = target
  t.update_state(cc, cs)
  cs.out.aEgo += 0.02 * (target * 0.8 - cs.out.aEgo)
  _, pa = t.pcm_request(cc, cs, target, 0.2, 2.0, NIDEC_GAS_MAX)
  if i % 10 == 0:
    t.note_pcm_tx(pa)
  if i % 2 == 0:
    t.update_pcm(cc, cs, target, NIDEC_GAS_MAX, False)
check("average_factor does not collapse to the identity under a real sequence",
      t.average_factor < 0.999, f"{t.average_factor:.6f}")

# Pitch must fade out at standstill: on an uphill the term asks for less brake,
# which is right while decelerating and wrong while holding the car on a hill.
t = make_tuner()
cc, cs = base(0.0, -0.5)
cc.orientationNED = [0.0, 0.10, 0.0]          # ~10% uphill
for _ in range(2000):
  hill_stopped = t.update_state(cc, cs)
check("pitch feedforward is zero at standstill", abs(hill_stopped) < 1e-9, f"{hill_stopped}")
cc.actuators.longControlState = LongCtrlState.stopping
cs.out.vEgo = 8.0
for _ in range(500):
  hill_stopping = t.update_state(cc, cs)
check("pitch feedforward is zero during stopping", abs(hill_stopping) < 1e-9, f"{hill_stopping}")
cc.actuators.longControlState = LongCtrlState.pid
for _ in range(2000):
  hill_moving = t.update_state(cc, cs)
check("pitch feedforward is live while cruising", hill_moving > 0.5, f"{hill_moving}")

# The crossfade must not touch a car that has no interceptor.
class _NoPedalCP_SP:
  enableGasInterceptor = False
dt._open_params = lambda: FakeParams(
  {"HondaDynamicTuningEnabled": True, "HondaDynamicPcmBlendEnabled": True}, known)
t = HondaDynamicTuner(CP=None, CP_SP=_NoPedalCP_SP())
check("crossfade stays off without a gas interceptor", t.pcm_blend is False)
check("crossfade off -> no authority anywhere", all(t.pcm_authority(v) == 0.0 for v in (10, 20, 40)))

# Params that come back as a numpy scalar / string must still load.
p = FakeParams({"HondaDynamicTuningEnabled": True,
                "HondaDynBrakeGain": np.float64(0.25),
                "HondaDynGasFactor": "1.25"}, known)
t = make_tuner(params=p)
check("numpy scalar param loads", abs(t.brake_gain_converged - 0.25) < 1e-9, f"{t.brake_gain_converged}")
check("string param loads", abs(t.gas_factor - 1.25) < 1e-9, f"{t.gas_factor}")
check("registry default of 0.0 means no day-one brake gain",
      make_tuner(params=FakeParams({"HondaDynamicTuningEnabled": True}, known)).brake_gain_converged == 0.0)


# --- 13. log-derived regressions --------------------------------------------

print("\n[13] log-derived regressions")

# Measured plant lag on the real routes is 0.23-0.34 s. Re-run the ramp case at
# the measured value as well as at the more pessimistic 0.5 s already covered.
for tau_s, alpha in (("0.30 s (measured)", 0.033), ("0.50 s (pessimistic)", 0.02)):
  t = make_tuner()
  cc, cs = base(15.0, 0.0, 0.0)
  peak, target = 1.0, 0.0
  for i in range(60000):
    want = -2.0 if (i // 3000) % 2 == 0 else 0.0
    target += float(np.clip(want - target, -0.02, 0.02))
    cc.actuators.accel = target
    t.update_state(cc, cs)
    cs.out.aEgo += alpha * (target - cs.out.aEgo)
    peak = max(peak, t.brake_gain(cc, cs, max(0.0, -target) * 0.2))
  check(f"lag of {tau_s} does not rail the brake gain",
        peak < 1.0 + BRAKE_POS_LIMIT - 0.05, f"peak={peak:.4f}")

# Small commands carry no identifiable gain information -- at settled cruise the
# residual is grade, not pedal gain (R^2 ~0.00 above 10 m/s in the logs).
t = make_tuner()
cc, cs = base(10.0, dt.LEARN_MIN_CMD - 0.05, -1.0)
settle(t, cc, cs)
before = list(t.pedal_gain)
for _ in range(5000):
  t.update_state(cc, cs)
  t.update_pedal(cc, cs, 0.5)
check("no pedal learning below the command floor", t.pedal_gain == before)

t = make_tuner()
cc, cs = base(10.0, -(dt.LEARN_MIN_CMD - 0.05), 0.5)
settle(t, cc, cs)
before = t.brake_pid.i
for _ in range(5000):
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.5)
check("no brake learning below the command floor", abs(t.brake_pid.i - before) < 1e-9)

t = make_tuner()
cc, cs = base(10.0, dt.LEARN_MIN_CMD + 0.4, -1.0)
settle(t, cc, cs)
before = list(t.pedal_gain)
for _ in range(5000):
  t.update_state(cc, cs)
  t.update_pedal(cc, cs, 0.5)
check("pedal learning still runs above the command floor", t.pedal_gain != before)

# The persisted estimate must never advance while the live value sits on a clamp,
# no matter how long the excursion lasts.
t = make_tuner()
cc, cs = base(10.0, 3.0, -6.0)
settle(t, cc, cs)
for _ in range(6000):
  t.update_state(cc, cs)
  t.update_pedal(cc, cs, 0.5)
check("pedal gain reached its rail", max(t.pedal_gain) >= PEDAL_GAIN_MAX - 1e-6)
conv_at_rail = max(t.pedal_gain_converged)
for _ in range(40000):
  t.update_state(cc, cs)
  t.update_pedal(cc, cs, 0.5)
check("converged estimate does not creep while the live value is railed",
      abs(max(t.pedal_gain_converged) - conv_at_rail) < 1e-6,
      f"{conv_at_rail:.4f} -> {max(t.pedal_gain_converged):.4f}")

t = make_tuner()
cc, cs = base(15.0, -2.0, 0.5)
settle(t, cc, cs)
for _ in range(40000):
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.9)
check("brake gain reached its rail", t.brake_pid_factor >= BRAKE_POS_LIMIT - 1e-6)
# the few frames spent climbing to the rail are legitimately off-rail, so a tiny
# amount leaks in; what matters is that it stays negligible and then stops
conv_at_rail = t.brake_gain_converged
check("railed brake gain stays negligible in the converged estimate",
      conv_at_rail < 0.05 * BRAKE_POS_LIMIT, f"{conv_at_rail:.5f} of {BRAKE_POS_LIMIT}")
for _ in range(80000):
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.9)
check("converged brake estimate stops advancing once railed",
      abs(t.brake_gain_converged - conv_at_rail) < 1e-9,
      f"{conv_at_rail:.6f} -> {t.brake_gain_converged:.6f}")

# Sustained, in-range learning SHOULD persist (the old |err|<0.3 gate blocked
# exactly this whenever a real bias was present).
t = make_tuner()
cc, cs = base(10.0, 1.0)
settle(t, cc, cs)
for _ in range(40000):
  t.update_state(cc, cs)
  cs.out.aEgo = cc.actuators.accel * 0.8 * t.pedal_gain_at(cs.out.vEgo)
  t.update_pedal(cc, cs, 0.5)
check("a real steady bias does reach the converged estimate",
      t.pedal_gain_converged[3] > 1.05,
      f"live={t.pedal_gain[3]:.3f} conv={t.pedal_gain_converged[3]:.3f}")


# --- 14. third-review regressions --------------------------------------------
#
# Each of these reproduced against the real module before the fix; they are the
# failure, not the feature. Do not relax one without re-deriving why it is here.

print("\n[14] third-review regressions")

# (a) nothing wound up in one engagement may cross into the next. brake_gain()'s
# only unwind paths were the stopping clamp and the standstill reset, and neither
# fires on a plain disengage at speed -- so a gain railed on a downgrade came back
# open-loop on the next engagement and stayed for the whole 1.5 s dwell.
t = make_tuner()
cc, cs = base(15.0, -1.5, -1.0)
settle(t, cc, cs)
for _ in range(2000):
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.5)
wound = t.brake_pid_factor
check("brake gain does wind up while learning", wound > 0.5 * BRAKE_POS_LIMIT, f"{wound:.4f}")

off_cc, off_cs = base(15.0, 0.0, 0.0)
off_cc.longActive = False
off_cc.actuators.longControlState = LongCtrlState.off
for _ in range(300):
  t.update_state(off_cc, off_cs)
  t.brake_gain(off_cc, off_cs, 0.0)
re_cc, re_cs = base(15.0, -0.5, 0.0)
first = t.brake_gain(re_cc, re_cs, 0.4)
check("a disengage unwinds the brake integrator to the converged estimate",
      abs(first - (1.0 + t.brake_gain_converged)) < 1e-9,
      f"first frame after re-engage {first:.4f}, converged says {1 + t.brake_gain_converged:.4f}")
check("the railed value does not survive the disengage",
      first < 1.0 + 0.1 * BRAKE_POS_LIMIT, f"{first:.4f}")

# (b) the standstill hold is the one brake command with no feedback, and it is
# what interface.py's stopAccel was hand-tuned against. A learned gain must not
# reach it -- 0.33 put the measured hold back at cb 251 and 0.50 railed it.
t = make_tuner()
t.brake_gain_converged = 0.5
t.brake_pid.i = t.brake_pid_factor = 0.5
hold_cc, hold_cs = base(0.0, -0.8, 0.0)
hold_cc.actuators.longControlState = LongCtrlState.stopping
t.update_state(hold_cc, hold_cs)
check("no learned gain is applied to the standstill hold",
      t.brake_gain(hold_cc, hold_cs, 0.75) == 1.0,
      f"{t.brake_gain(hold_cc, hold_cs, 0.75):.4f}")
check("the standstill reset still re-arms the estimate for the next stop",
      abs(t.brake_pid.i - t.brake_gain_converged) < 1e-9, f"{t.brake_pid.i:.4f}")
# ... and the gain is still live on the approach, which is where it earns its keep
t2 = make_tuner()
t2.brake_gain_converged = 0.5
t2.brake_pid.i = t2.brake_pid_factor = 0.5
app_cc, app_cs = base(3.0, -1.5, -1.0)
app_cc.actuators.longControlState = LongCtrlState.stopping
t2.update_state(app_cc, app_cs)
check("the gain is still applied while still moving in the stopping phase",
      t2.brake_gain(app_cc, app_cs, 0.6) > 1.4, f"{t2.brake_gain(app_cc, app_cs, 0.6):.4f}")

# (c) the PCM channel was the only learner without LEARN_MIN_CMD. gas_alpha is an
# unweighted bias, so at settled cruise it integrated the grade residual straight
# to its rail -- and persisted it, giving the next drive a standing gas request.
t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 0.05, -0.25)          # 0.05 target, 0.30 shortfall
settle(t, cc, cs)
for _ in range(20000):
  t.update_state(cc, cs)
  t.pcm_request(cc, cs, 0.05, 0.19, 0.6, NIDEC_GAS_MAX)
  t.note_pcm_tx(t.new_accel)
  t.update_pcm(cc, cs, 0.05, NIDEC_GAS_MAX, False)
check("no pcm learning below the command floor",
      t.gas_alpha == 0.0 and t.gas_factor == 1.0,
      f"alpha={t.gas_alpha:.5f} factor={t.gas_factor:.5f}")
# above the floor it must still learn, or the gate is just an off switch
t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 1.0, 0.7)
settle(t, cc, cs)
for _ in range(20000):
  t.update_state(cc, cs)
  t.pcm_request(cc, cs, 1.0, 0.19, 0.6, NIDEC_GAS_MAX)
  t.note_pcm_tx(t.new_accel)
  t.update_pcm(cc, cs, 1.0, NIDEC_GAS_MAX, False)
check("pcm learning still runs above the command floor",
      t.gas_alpha > 0.0, f"alpha={t.gas_alpha:.5f}")

# (d) gas_factor multiplies gas_accel, so its gradient must be weighted by
# gas_accel. Weighted by adjust_accel it inverts wherever adjust_accel < 0 <
# gas_accel; that band is now unreachable behind the floor, but the weight has to
# be the right one regardless.
t = make_tuner(pcm_blend=True)
cc, cs = base(25.0, 1.0, 0.7)
settle(t, cc, cs)
t.pcm_request(cc, cs, 1.0, 0.19, 0.6, NIDEC_GAS_MAX)
check("gas_factor's gradient weight is the term it actually multiplies",
      abs(t.gas_accel_applied - (1.0 + 0.19 * t.wind_factor)) < 1e-9,
      f"{t.gas_accel_applied:.4f} vs adjust_accel 1.0")
t.reset_pcm_feedforward()
check("the stashed weight is cleared with the rest of the feedforward state",
      t.gas_accel_applied == 0.0, f"{t.gas_accel_applied}")

# (e) PEDAL_GAIN_BP is ELESYS_GAS_BP, which lives in another file. Growing it
# without adding the matching param must not take out CarController.__init__.
t = make_tuner()
try:
  v = t._get_float("HondaDynPedalGain9")
  check("an unregistered key degrades instead of raising", v == 1.0, f"got {v}")
except Exception as e:
  check("an unregistered key degrades instead of raising", False, f"raised {type(e).__name__}")


# --- 15. drive-mode gating ----------------------------------------------------
#
# S holds lower gears, so the same interceptor command lands a different accel.
# One set of tables pooled across modes converges on a blend that matches no
# actual driving, which is worse than not learning.

print("\n[15] drive-mode gating")

GearShifter = structs.CarState.GearShifter


def learn_in(gear, frames=6000):
  t = make_tuner()
  cc, cs = base(10.0, 1.0, 0.7)
  cs.out.gearShifter = gear
  settle(t, cc, cs)
  for _ in range(frames):
    t.update_state(cc, cs)
    t.update_pedal(cc, cs, 0.5)
    t.brake_gain(cc, cs, 0.0)
  return t


t_d = learn_in(GearShifter.drive)
check("learning runs in D", t_d.pedal_gain[3] > 1.0, f"{t_d.pedal_gain[3]:.4f}")
t_s = learn_in(GearShifter.sport)
check("learning is frozen in S",
      all(g == 1.0 for g in t_s.pedal_gain), f"{[round(g, 4) for g in t_s.pedal_gain]}")
check("and mode_ok reports why", not t_s.mode_ok and t_s.drive_mode == ("sport", None),
      f"mode_ok={t_s.mode_ok} mode={t_s.drive_mode}")

# ECON is not mapped on this car yet, so it must stay a no-op: mode is (gear, None)
# and nothing gates on it. This pins the hook so wiring it later is a visible change.
check("ECON is unmapped and gates nothing today",
      t_d.drive_mode == ("drive", None) and dt.HondaDynamicTuner._econ_state(base(10.0)[1]) is None,
      f"{t_d.drive_mode}")
check("a mapped ECON would gate (LEARN_ECON is False-only)",
      not dt.HondaDynamicTuner._mode_learnable(("drive", True))
      and dt.HondaDynamicTuner._mode_learnable(("drive", False))
      and dt.HondaDynamicTuner._mode_learnable(("drive", None)))

# unknown gear must NOT gate -- every other Nidec car leaves gearShifter unknown,
# and gating there would silently disable the whole feature on those platforms
t_u = learn_in(GearShifter.unknown)
check("unknown gear still learns (other Nidec platforms)",
      t_u.pedal_gain[3] > 1.0 and t_u.mode_ok, f"{t_u.pedal_gain[3]:.4f} mode_ok={t_u.mode_ok}")

# brake channel is gated too
t = make_tuner()
cc, cs = base(15.0, -1.5, -1.0)
cs.out.gearShifter = GearShifter.sport
settle(t, cc, cs)
for _ in range(3000):
  t.update_state(cc, cs)
  t.brake_gain(cc, cs, 0.5)
check("brake learning is frozen in S too", t.brake_pid_factor == 0.0, f"{t.brake_pid_factor:.4f}")

# a mode change is a transient: the dwell must re-arm, not carry across
t = make_tuner()
cc, cs = base(10.0, 1.0, 0.7)
cs.out.gearShifter = GearShifter.drive
settle(t, cc, cs)
check("settled in D before the shift", t._settle >= SETTLE_FRAMES, f"{t._settle}")
cs.out.gearShifter = GearShifter.sport
t.update_state(cc, cs)
check("a gear change resets the dwell", t._settle == 0, f"{t._settle}")
cs.out.gearShifter = GearShifter.drive
t.update_state(cc, cs)
check("and again on the way back", t._settle == 0, f"{t._settle}")


# --- 16. aero learner is confined to where it is identifiable ------------------
#
# Chained replay over the 13 engaged drives had wind_factor ending at the 1.500
# clamp on 6 of them. The update is symmetric in form, but the accel error on this
# car is not zero-mean (-0.34..-0.47 on gas), so it drifted one way -- against a
# bias the pedal gain and pitch feedforward were already chasing.

print("\n[16] wind learner is disjoint from the pedal/brake learners")

# in the pedal learner's region: wind must not move
t = make_tuner()
cc, cs = base(10.0, 1.0, 0.3)          # accel_target 1.0 >= LEARN_MIN_CMD
settle(t, cc, cs)
w0 = t.wind_factor
for _ in range(20000):
  t.update_state(cc, cs)
  t.update_wind(cc, cs, 0.19)
check("no aero learning where the pedal learner owns the command",
      t.wind_factor == w0, f"{w0:.4f} -> {t.wind_factor:.4f}")

# in the brake learner's region: also must not move
t = make_tuner()
cc, cs = base(10.0, -1.0, -0.5)
settle(t, cc, cs)
w0 = t.wind_factor
for _ in range(20000):
  t.update_state(cc, cs)
  t.update_wind(cc, cs, 0.19)
check("nor where the brake learner owns it", t.wind_factor == w0,
      f"{w0:.4f} -> {t.wind_factor:.4f}")

# near-zero command with a real error: this IS its region, it must still learn
t = make_tuner()
cc, cs = base(25.0, 0.1, -0.2)         # |target| < LEARN_MIN_CMD, err = +0.3
settle(t, cc, cs)
w0 = t.wind_factor
for _ in range(20000):
  t.update_state(cc, cs)
  t.update_wind(cc, cs, 0.19)
check("but it does learn in the cruise band", t.wind_factor > w0 + 1e-6,
      f"{w0:.4f} -> {t.wind_factor:.4f}")

# noise deadband: tiny errors must not ratchet
t = make_tuner()
cc, cs = base(25.0, 0.1, 0.1 - 0.02)   # err = +0.02, inside WIND_ERR_DEADBAND
settle(t, cc, cs)
w0 = t.wind_factor
for _ in range(20000):
  t.update_state(cc, cs)
  t.update_wind(cc, cs, 0.19)
check("sub-deadband error does not ratchet the aero term",
      t.wind_factor == w0, f"{w0:.4f} -> {t.wind_factor:.4f} (deadband {dt.WIND_ERR_DEADBAND})")


print("\n" + "=" * 60)
if FAILURES:
  print(f"{len(FAILURES)} FAILED: {FAILURES}")
  sys.exit(1)
print("ALL CHECKS PASSED")
