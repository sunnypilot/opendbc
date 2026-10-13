"""
Vehicle + PCM stand-in for the Toyota long-control harness.

The carcontroller carries features aimed at specific things the car does. A plant without
those behaviours cannot tell whether the features work - it reports "smooth" while being
blind to the symptom. Three are modelled here:

* PCM following - slew limit, dead time, first-order lag, road load, and what the car can
  actually deliver (tighter than the ACCEL_MIN/MAX command clip).
* Regen / friction hand-over - past the regen limit the PCM cuts the friction brake below
  about 18 km/h, so the car delivers LESS than asked over roughly 15-4 km/h (worst around
  5-8) and a little MORE near the end of the stop once the friction is back. Light
  requests only: on a firm request the regen gives more, not less.
* Friction entry - the friction brake comes in past the regen limit and how hard it bites
  depends on how fast the command was moving at that moment, so a fast command rate at
  entry produces a jolt in the delivered decel.

Parameters are read off the logged stops recorded in the brake-onset history. They are a
model built from those numbers, not a fit to them: the harness is evidence about the
loop's behaviour and about which feature moves which symptom, never about the car.
"""

import numpy as np

KMH = 1.0 / 3.6

# PCM following
TAU = 0.30
DEAD_TIME = 0.10
PCM_RATE = 4.0  # m/s^3, matches the rate of ACCEL_NET the carcontroller's limit is set to
DRAG = 0.005
# What the car can deliver. The regen contribution is added on top of this by the
# over-delivery term below, which is how the car ends up past DELIVER_MIN at all.
DELIVER_MIN = -3.5
DELIVER_MAX = 1.5

# Regen / friction hand-over shortfall: a fraction of the request, by speed (m/s).
# Request / delivered pairs from the logged stops behind the hand-over feed-forward:
#   -0.84/-0.61 and -0.87/-0.69 at 11-12 km/h, -1.04/-0.81 at 13,
#   -0.75/-0.62, -0.89/-0.58, -0.77/-0.54 at 8-7, -0.63/-0.45, -0.75/-0.50 at 6-5,
#   -0.60/-0.40, -0.67/-0.28 at 5-4, back on target by 3.
HANDOVER_V_BP = [2.0 * KMH, 3.5 * KMH, 5.0 * KMH, 7.0 * KMH, 9.0 * KMH, 12.0 * KMH, 15.0 * KMH, 18.0 * KMH]
HANDOVER_FRAC = [0.00, 0.00, 0.30, 0.22, 0.20, 0.24, 0.27, 0.00]
HANDOVER_LIGHT_REQ = -1.0  # at or lighter than this the shortfall is full
HANDOVER_FIRM_REQ = -1.5   # at or firmer there is none

# Over-delivery: the car adds decel of its own, growing with the request.
#   asked -1.46 -> delivered -1.72, asked -2.26 -> -2.74, asked -3.49 -> -4.30 (0.8 s)
OVERDELIVER_MIN_REQ = -1.0
OVERDELIVER_GAIN = 0.45
OVERDELIVER_MAX = 0.8

# Friction brake entry.
FRICTION_ENTRY_REQ = -0.6     # past the regen limit the friction brake comes in
FRICTION_RELEASE_REQ = -0.2
FRICTION_FORCE_FULL = 1500.0  # N once established
# Force rate against command rate at entry: 3067/1067/533 N/s at 1.91/1.05/0.44 m/s^3.
FRICTION_FORCE_RATE = 1600.0  # N/s per m/s^3 of command rate at entry
# Delivered jerk against command rate at entry: 3.70/2.22/1.22 at 1.91/1.05/0.44 m/s^3.
FRICTION_JERK_GAIN = 2.1      # delivered jerk per m/s^3 of command rate at entry


class ToyotaLongPlant:
  def __init__(self, tau=TAU, dead_time=DEAD_TIME, pcm_rate=PCM_RATE, drag=DRAG,
               deliver_min=DELIVER_MIN, deliver_max=DELIVER_MAX, dt=0.03):
    self.tau = tau
    self.dead_time = dead_time
    self.pcm_rate = pcm_rate
    self.drag = drag
    self.deliver_min = deliver_min
    self.deliver_max = deliver_max
    self.dt = dt
    self.reset(0.0, 0.0)

  def reset(self, v_ego, a_ego):
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.a_pcm = a_ego
    self.prev_cmd = a_ego
    self._dead = [a_ego] * max(1, round(self.dead_time / self.dt))
    self.friction_in = False
    self.brake_force = 0.0
    self.entry_jerk = self.pcm_rate
    self.entry_rate = 0.0

  def _handover(self, a_delivered, a_cmd, v_ego):
    """What the car does beyond simply following the command."""
    if a_cmd >= 0.0:
      return a_delivered
    # friction cut below ~18 km/h: a light request is not delivered in full, so the
    # delivered decel is LIGHTER than asked (toward zero), which is the dip the driver
    # feels as the brakes going light
    light = float(np.clip((HANDOVER_FIRM_REQ - a_cmd) / (HANDOVER_FIRM_REQ - HANDOVER_LIGHT_REQ), 0.0, 1.0))
    shortfall = float(np.interp(v_ego, HANDOVER_V_BP, HANDOVER_FRAC)) * light * -a_cmd
    # regen on top: a firm request is delivered with MORE decel than asked
    over = float(np.clip(OVERDELIVER_GAIN * (-a_cmd + OVERDELIVER_MIN_REQ), 0.0, OVERDELIVER_MAX))
    return a_delivered + shortfall - over

  def _friction(self, a_cmd):
    """Friction brake entry: how fast its force builds, and the jolt that comes with it."""
    cmd_rate = (a_cmd - self.prev_cmd) / self.dt
    self.prev_cmd = a_cmd

    if a_cmd > FRICTION_RELEASE_REQ:
      self.friction_in = False
      self.brake_force = max(0.0, self.brake_force - FRICTION_FORCE_RATE * self.dt)
      return self.pcm_rate

    if not self.friction_in:
      self.friction_in = True
      self.entry_jerk = FRICTION_JERK_GAIN * abs(cmd_rate)
      self.entry_rate = FRICTION_FORCE_RATE * abs(cmd_rate)

    self.brake_force = min(FRICTION_FORCE_FULL, self.brake_force + self.entry_rate * self.dt)
    establishing = self.brake_force < FRICTION_FORCE_FULL
    # while the force is still coming in the car's decel can move faster than the PCM's
    # own slew, at the rate the command was moving when the friction came in
    return self.entry_jerk if establishing else self.pcm_rate

  def step(self, a_cmd, v_hold):
    """One long-control tick. Returns the new (a_ego, v_ego)."""
    down_slew = self._friction(a_cmd)

    # first-order lag toward the request, then the slew limit applied to the actual
    # movement (clamping the setpoint first would divide the slew limit by tau)
    requested = self.a_pcm + (a_cmd - self.a_pcm) * self.dt / self.tau
    self.a_pcm = float(np.clip(requested, self.a_pcm - down_slew * self.dt,
                               self.a_pcm + self.pcm_rate * self.dt))

    # dead time on the delivered acceleration
    self._dead.append(self.a_pcm)
    a_delivered = self._dead.pop(0)

    # clip what the car can do from the request, then let the hand-over add or take away
    # on top - clipping after would swallow exactly the over-delivery we are modelling
    a_base = float(np.clip(a_delivered, self.deliver_min, self.deliver_max))
    self.a_ego = self._handover(a_base, a_cmd, self.v_ego) - self.drag * self.v_ego
    if not v_hold:
      self.v_ego = max(0.0, self.v_ego + self.a_ego * self.dt)
    return self.a_ego, self.v_ego
