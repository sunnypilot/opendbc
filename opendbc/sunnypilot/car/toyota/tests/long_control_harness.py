"""
Deterministic harness for Toyota longitudinal control.

It drives the *real* `CarController` (not a re-implementation), so any tuning change
is exercised through the same PID, rate limits, pitch feed-forward and clipping that
run on the car. A simple plant stands in for the PCM + vehicle so the loop can be
closed and measured.

The long path only runs every 3rd frame (33 Hz), matching ACC_CONTROL's rate.

The plant is a model, not the vehicle: numbers from this harness are evidence about
the loop's behaviour, not about the car. On-car logs remain the reference.
"""

from types import SimpleNamespace

import numpy as np
from opendbc.car import Bus, structs
from opendbc.car.toyota.carcontroller import CarController
from opendbc.car.toyota.values import CAR, ToyotaFlags
from opendbc.sunnypilot.car.toyota.tests.long_control_plant import ToyotaLongPlant
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

DT_CTRL = 0.01
LONG_DIV = 3  # long control runs every 3rd frame
LONG_DT = DT_CTRL * LONG_DIV

LongCtrlState = structs.CarControl.Actuators.LongControlState


class Trace:
  """One step response, clocked from the moment the request changes."""

  def __init__(self, a_from, a_to, target, v_ego):
    self.a_from = a_from
    self.a_to = a_to
    self.target = target
    self.v_ego = v_ego
    self.t = []
    self.a_cmd = []
    self.a_ego = []
    self.speed = []
    self.integral = []

  def array(self, name):
    return np.asarray(getattr(self, name), dtype=float)

  def _since(self, t_from):
    t = self.array("t")
    return t >= t_from

  def onset_rate(self):
    """Peak rate the command first moves at - the 'how hard does it bite' number."""
    j = np.diff(np.concatenate([[self.a_from], self.array("a_cmd")])) / LONG_DT
    return float(np.min(j) if self.a_to < self.a_from else np.max(j))

  def rise_time(self, frac=0.9):
    """Time for a_ego to cover `frac` of the step in the right direction."""
    span = self.target - self.a_from
    t = self.array("t")
    a = self.array("a_ego")
    done = a if span > 0 else -a
    goal = (self.a_from + frac * span) if span > 0 else -(self.a_from + frac * span)
    hit = np.nonzero(done >= goal)[0]
    return float(t[hit[0]]) if hit.size else float("inf")

  def settle_time(self, band=0.1, hold=0.5):
    """Time until a_ego stays within `band` of target for `hold` seconds."""
    t = self.array("t")
    inside = np.abs(self.array("a_ego") - self.target) <= band
    n = max(1, int(round(hold / LONG_DT)))
    for i in range(len(inside)):
      if inside[i:i + n].all():
        return float(t[i])
    return float("inf")

  def overshoot(self, t_from=0.0):
    """Excursion past target, in the direction of travel (0.0 if none)."""
    a = self.array("a_ego")[self._since(t_from)]
    if self.a_to < self.a_from:
      return max(0.0, self.target - float(a.min()))
    return max(0.0, float(a.max()) - self.target)

  def steady_state(self, t_from):
    m = self._since(t_from)
    return float(self.array("a_cmd")[m].mean()), float(self.array("a_ego")[m].mean())

  def tracking_error(self, t_from):
    """Requested minus achieved accel.

    Positive means the car delivers more deceleration (or less acceleration) than
    asked. This is deliberately *not* `a_cmd - a_ego`: the command legitimately
    differs from the request by whatever the integral is trimming, so measuring
    against the command would hide exactly the bias we care about.
    """
    m = self._since(t_from)
    return self.a_to - float(self.array("a_ego")[m].mean())

  def command_bias(self, t_from):
    """Command minus request: how much the integral is trimming."""
    m = self._since(t_from)
    return float(self.array("a_cmd")[m].mean()) - self.a_to

  def jerk(self):
    """Rate of change of the achieved accel, m/s^3."""
    return np.diff(np.concatenate([[self.a_from], self.array("a_ego")])) / LONG_DT

  def max_jerk(self):
    return float(np.abs(self.jerk()).max())

  def onset_jerk(self, window=0.5):
    """Largest jerk of the achieved decel within `window` seconds of the step.

    This is the number that says how hard the brake bites: a soft onset keeps it low
    even when the request is large.
    """
    n = max(1, int(round(window / LONG_DT)))
    return float(np.abs(self.jerk()[:n]).max())

  def peak_accel(self):
    """Most negative achieved accel - how much braking the car actually produced."""
    return float(self.array("a_ego").min())

  def time_to(self, level):
    """First time the achieved accel reaches `level` (e.g. -2.8 m/s^2)."""
    a = self.array("a_ego")
    t = self.array("t")
    hit = np.nonzero((a <= level) if level < 0 else (a >= level))[0]
    return float(t[hit[0]]) if hit.size else float("inf")

  def _moving(self, min_speed=3.0 / 3.6):
    """Samples above `min_speed`. Below it the car is stopping, so it cannot follow the

    command at all and every delivery metric would just report the standstill.
    """
    return self.array("speed") > min_speed

  # The delivery metrics all compare against the REQUEST (`a_to`), never the command.
  # The command is the feedforward plus the integrator plus whatever corrections are
  # active, so measuring against it would hide the very thing a correction does: move the
  # command. What the driver feels is what the car delivers against what openpilot asked.
  def over_delivery(self, min_speed=3.0 / 3.6):
    """Furthest the car braked past the request (positive = harder than asked).

    This is what a brake-overshoot limiter takes back.
    """
    m = self._moving(min_speed)
    return float(np.max((self.a_to - self.array("a_ego"))[m]))

  def shortfall(self, min_speed=3.0 / 3.6):
    """Furthest the car fell short of the request (positive = lighter than asked).

    Note this also picks up any transient where the request steps away faster than the
    car can follow, so it is not specific to the low-speed hand-over. Use `band_gap` for
    a speed-scoped measure.
    """
    m = self._moving(min_speed)
    return float(np.max((self.array("a_ego") - self.a_to)[m]))

  def band_gap(self, lo_kmh, hi_kmh):
    """Mean (delivered - requested) while the speed sits in (hi, lo] km/h.

    Positive = the car is braking lighter than asked. Scoping by speed keeps the onset
    transient out: a band this low is only reached long after the request started.
    """
    speed = self.array("speed") * 3.6
    m = (speed <= lo_kmh) & (speed > hi_kmh)
    return float((self.array("a_ego")[m] - self.a_to).mean())

  def delivery_wander(self, min_speed=3.0 / 3.6):
    """Peak-to-peak of (request - delivered) while moving: how much the delivery moves

    relative to what was asked. A hand-over dip followed by the friction returning shows
    up here as the 'brakes, goes light, brakes again' the driver reports.
    """
    m = self._moving(min_speed)
    return float(np.ptp((self.a_to - self.array("a_ego"))[m]))

  def speed_at(self, t):
    """Ego speed at time `t` (m/s)."""
    return float(np.interp(t, self.array("t"), self.array("speed")))

  def command_ripple(self, t_from):
    """Peak-to-peak command movement over the tail - smoothness of the output."""
    return float(np.ptp(self.array("a_cmd")[self._since(t_from)]))

  def report(self, tail=1.0):
    t_end = self.t[-1]
    return {
      "v_kmh": self.v_ego * 3.6,
      "from": self.a_from,
      "to": self.a_to,
      "onset_rate": self.onset_rate(),
      "rise_time": self.rise_time(),
      "overshoot": self.overshoot(),
      "settle_time": self.settle_time(),
      "tracking_error": self.tracking_error(t_end - tail),
      "ripple": self.command_ripple(t_end - tail),
    }


class LongHarness:
  """Drives the real CarController's long path with a synthetic CarState/CarControl."""

  def __init__(self, car=CAR.TOYOTA_COROLLA_TSS2, flags=None, plant=None, pitch=0.0, brake_onset=True):
    self.CP = structs.CarParams(brand="toyota", carFingerprint=str(car),
                                flags=int(ToyotaFlags.TSS2 if flags is None else flags),
                                openpilotLongitudinalControl=True)
    self.CP_SP = structs.CarParamsSP()
    # the brake-onset module is param-gated on the car; without this the harness silently
    # exercises the stock path and any onset assertion passes for the wrong reason.
    # `brake_onset=False` builds the stock baseline, so a correction can be measured on/off.
    self.brake_onset = brake_onset
    if brake_onset:
      self.CP_SP.flags |= ToyotaFlagsSP.SP_BRAKE_ONSET.value

    dbc = "toyota_secoc_pt_generated" if (self.CP.flags & ToyotaFlags.SECOC) else "toyota_nodsu_pt_generated"
    self.dbc_names = {Bus.pt: dbc, Bus.radar: "toyota_tss2_adas"}
    self.pitch = pitch
    self.plant = ToyotaLongPlant() if plant is None else plant

    self.CS = SimpleNamespace(
      out=structs.CarState.new_message(),
      gvc=0.0,
      acc_type=1,
      pcm_follow_distance=0,
      pcm_acc_status=5,
      lkas_hud={},
      secoc_synchronization=None,
      brake_force=0.0,
      gasPressed=False,
      brakePressed=False,
      steeringPressed=False,
    )
    self.cc = None
    self.CC = None
    self.CC_SP = structs.CarControlSP()
    self.reset(0.0, 0.0)

  def _init_control(self):
    self.CC = structs.CarControl.new_message()
    self.CC.longActive = True
    self.CC.latActive = False
    self.CC.init("actuators")
    self.CC.actuators.longControlState = LongCtrlState.pid
    self.CC.actuators.torque = 0.0
    self.CC.actuators.steeringAngleDeg = 0.0
    self.CC.cruiseControl.cancel = False
    self.CC.init("hudControl")
    self.CC.hudControl.visualAlert = structs.CarControl.HUDControl.VisualAlert.none
    self.CC.hudControl.leadVisible = False
    self.CC.hudControl.leadDistanceBars = 2
    self.CC.orientationNED = [0.0, float(self.pitch), 0.0]
    self.CC_SP.enableGasInterceptor = False

  def reset(self, v_ego, a_ego=0.0):
    self.cc = CarController(self.dbc_names, self.CP, self.CP_SP)
    self.plant.reset(v_ego, a_ego)
    self._init_control()

  def frame(self, accel, long_active=True):
    """One 100 Hz CarController.update(). Returns the accel command sent to the PCM."""
    cs = self.CS.out
    v_ego, a_ego = self.plant.v_ego, self.plant.a_ego
    cs.vEgo = v_ego
    cs.vEgoRaw = v_ego
    cs.aEgo = a_ego
    cs.gasPressed = False
    cs.brakePressed = False
    cs.standstill = v_ego < 1e-3
    cs.steeringTorque = 0.0
    cs.steeringTorqueEps = 0.0
    cs.steeringRateDeg = 0.0
    cs.steeringAngleDeg = 0.0
    cs.steeringAngleOffsetDeg = 0.0
    cs.cruiseState.enabled = long_active

    self.CC.longActive = long_active
    self.CC.actuators.accel = accel
    self.CC.actuators.longControlState = LongCtrlState.pid if long_active else LongCtrlState.off
    self.CS.gvc = a_ego
    self.CS.brake_force = self.plant.brake_force  # CarState, not CarState.out

    # the real controller reads CC as a cereal message (a capnp reader) and calls
    # actuators.as_builder() on it, so hand it a reader view rather than the builder
    new_actuators, _ = self.cc.update(self.CC.as_reader(), self.CC_SP, self.CS, 0)
    return new_actuators.accel

  def tick(self, accel, v_hold=True):
    """One long-control tick: 3 frames, plant advances once."""
    cmd = accel
    for _ in range(LONG_DIV):
      cmd = self.frame(accel)
    self.plant.step(cmd, v_hold)
    return cmd

  def _record(self, tr, accel, v_hold, seconds):
    for i in range(int(round(seconds / LONG_DT))):
      tr.t.append(i * LONG_DT)
      tr.a_cmd.append(self.tick(accel, v_hold=v_hold))
      tr.a_ego.append(self.plant.a_ego)
      tr.speed.append(self.plant.v_ego)
      tr.integral.append(self.cc.long_pid.i)
    return tr

  def stop(self, request, v_ego, seconds=40.0):
    """Hold a brake request from a cruise down to standstill, speed free to fall.

    The low-speed regen hand-over, the friction entry and the end-of-stop delivery all
    live in this run, which is where 'brakes, goes light, brakes again' and 'the last two
    metres feel light' show up.
    """
    self.reset(v_ego, 0.0)
    tr = Trace(0.0, request, request, v_ego)
    for i in range(int(round(seconds / LONG_DT))):
      tr.t.append(i * LONG_DT)
      tr.a_cmd.append(self.tick(request, v_hold=False))
      tr.a_ego.append(self.plant.a_ego)
      tr.speed.append(self.plant.v_ego)
      tr.integral.append(self.cc.long_pid.i)
      if self.plant.v_ego <= 1e-3:
        break
    return tr

  def hold(self, accel, v_ego, seconds, v_hold=True):
    """Constant request from rest at a fixed speed - shows settling and limit cycles."""
    self.reset(v_ego, 0.0)
    return self._record(Trace(0.0, accel, accel, v_ego), accel, v_hold, seconds)

  def step(self, a_from, a_to, v_ego, seconds=8.0, v_hold=True, warmup=3.0):
    """Settle at `a_from`, then step to `a_to`, at a fixed speed.

    The returned trace is clocked from the step itself, so the settling metrics are
    about the step and not the warm-up.
    """
    self.reset(v_ego, a_from)
    self._record(Trace(a_from, a_from, a_from, v_ego), a_from, v_hold, warmup)
    return self._record(Trace(a_from, a_to, a_to, v_ego), a_to, v_hold, seconds)
