"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

ONSET_T_BP = [0.0, 0.15, 0.6]  # s
# Speed schedule of the brake onset. Below 10 km/h (creep follow: the lead moves a little, the car catches up and
# stops again) the Altis PCM bites hard on a modest request (route 000000ed 04:05:41: output -0.42 -> -1.25 m/s^2,
# 1400 N; five ACC re-stops on 10-03/04 all -0.9..-1.25), so at creep speeds the onset uses the same numbers as the
# engage schedule (driver 2026-10-04 "yes, apply it"): 0.5 m/s^3 for 0.2 s, stock by 0.7 s; from 20 km/h the normal
# 1.0 m/s^3 for 0.1 s, stock by 0.4 s (driver 2026-10-03), interpolated between.
# Driver 2026-10-04 after route 000000f2: "the brake after creeping forward again improved but is still not soft
# enough, softer again" -> below 10 km/h 0.3 m/s^3 for 0.3 s (-0.09 m/s^2 at 0.3 s), stock by 0.9 s.
# Driver 2026-10-05: "at 50 km/h the brake still feels heavy and there was plenty of distance; every brake should start
# extremely light, and if that needs distance, start earlier" -> the creep-speed numbers (0.3 m/s^3 for 0.3 s, stock
# by 0.9 s) now apply at every speed up to 60 km/h, fading to the highway onset (1.0 m/s^3, 0.1 s, stock by 0.4 s) at
# 80 km/h, which stays as it was. The planner's light-onset margin (long_mpc) supplies the extra distance.
# Driver 2026-10-05, same drive: "at 40-50 km/h the build from light to firm is still not smooth, a bit abrupt" -> the
# jerk the onset ramps UP TO is also scheduled: 2.0 m/s^3 up to 60 km/h (was the stock 4.0 from 0.9 s on), stock at 80.
ONSET_V_BP = [16.7, 22.2]  # m/s (60, 80 km/h)
ONSET_T1_V = [0.3, 0.1]
ONSET_T3_V = [0.9, 0.4]
ONSET_J1_V = [0.3, 1.0]  # m/s^3 during the first phase
ONSET_J3_V = [2.0, 4.0]  # m/s^3 the ramp ends at (the shaper never exceeds the stock limit)
ONSET_J_DOWN = [1.0, 1.0, 4.0]  # m/s^3 (reference shape; the first two entries follow ONSET_J1_V)
# Engaging ACC (SET-) while creeping up to a lead (driver 2026-10-04: "the car used to brake hard the moment I press
# SET-; the first ~0.2 s must be a light touch, then blend into the decel the speed needs"). Route 000000ee 11:48:07:
# engaged at 9 km/h 4 m behind a 4 km/h lead, the request went 0 -> -0.42 in 0.2 s and -1.6 by 0.8 s, the car hit
# -2.7 m/s^2 (3280 N). While the engage window is open the down jerk is capped on time SINCE ENGAGING: 0.5 m/s^3 for
# the first 0.2 s (-0.1 m/s^2 at 0.2 s), then rising to stock by 0.7 s; the planner's full request still arrives, just
# later. The normal onset schedule applies on top (the stricter of the two wins).
ENGAGE_BRAKE_T_BP = [0.0, 0.2, 0.7]  # s since engaging
ENGAGE_BRAKE_J_DOWN = [0.5, 0.5, 4.0]  # m/s^3
HARD_BRAKE_ACCEL = -3.5  # driver 2026-10-05: "let the -2.0 bypass not act, I will test carefully" - at the ACCEL_MIN clip it never fires; only the FCW bypass remains (was -2.0; -1.5 before 2026-10-03)
URGENT_T = 0.1  # s
URGENT_J = 1.0  # m/s^3
URGENT_T_RAMP = 0.1  # s


class BrakeOnsetShaper:
  def __init__(self, dt: float, stock_down_jerk: float):
    self.dt = dt
    self.stock_down_jerk = stock_down_jerk
    self.t_onset = 0.0

  def reset(self) -> None:
    self.t_onset = 0.0

  @staticmethod
  def schedule_t(v_ego: float) -> list[float]:
    t1 = float(np.interp(v_ego, ONSET_V_BP, ONSET_T1_V))
    t3 = float(np.interp(v_ego, ONSET_V_BP, ONSET_T3_V))
    return [0.0, t1, t3]

  @staticmethod
  def schedule_j(v_ego: float) -> list[float]:
    j1 = float(np.interp(v_ego, ONSET_V_BP, ONSET_J1_V))
    j3 = float(np.interp(v_ego, ONSET_V_BP, ONSET_J3_V))
    return [j1, j1, j3]

  def down_step(self, accel_request: float, prev_accel: float, bypass: bool = False, v_ego: float = 30.0,
                urgent: bool = False, t_engaged: float | None = None) -> float:
    if bypass:
      self.t_onset = 0.0
      return -self.stock_down_jerk * self.dt

    if urgent:
      t_bp, j_bp = [0.0, URGENT_T, URGENT_T + max(URGENT_T_RAMP, self.dt)], [URGENT_J, URGENT_J, self.stock_down_jerk]
    else:
      t_bp, j_bp = self.schedule_t(v_ego), self.schedule_j(v_ego)
    gentlest_step = -ONSET_J_DOWN[0] * self.dt
    onset = (accel_request - prev_accel) < gentlest_step - 1e-9
    if onset:
      j_down = float(np.interp(self.t_onset, t_bp, j_bp))
      self.t_onset = min(self.t_onset + self.dt, t_bp[-1])
    else:
      j_down = self.stock_down_jerk
      self.t_onset = max(self.t_onset - self.dt, 0.0)
    if t_engaged is not None and t_engaged < ENGAGE_BRAKE_T_BP[-1]:
      j_down = min(j_down, float(np.interp(t_engaged, ENGAGE_BRAKE_T_BP, ENGAGE_BRAKE_J_DOWN)))
    return -min(j_down, self.stock_down_jerk) * self.dt

  def is_urgent(self, accel_request: float, fcw: bool) -> bool:
    return fcw or accel_request < HARD_BRAKE_ACCEL

ENGAGE_T_BP = [0.0, 0.1, 0.6]  # s; the window is still used to keep the soft BRAKE onset for a hard request after engaging
# Driver 2026-10-04: "if Kumar's base added a soft start again, remove it - I want an immediate response, not a lurch,
# following the eco/normal/sport profile". The gas-side engage ramp (0.25 -> 0.6 -> 4.0 m/s^3 over 0.6 s) is gone:
# the up jerk after engaging is the stock windup limit; the accel profile sets how much.
ENGAGE_J_UP = [4.0, 4.0, 4.0]  # m/s^3


class EngageOnsetShaper:
  def __init__(self, dt: float, stock_up_jerk: float):
    self.dt = dt
    self.stock_up_jerk = stock_up_jerk
    self.t_engaged: float | None = None

  def reset(self) -> None:
    self.t_engaged = None

  @property
  def in_engage_window(self) -> bool:
    return self.t_engaged is None or self.t_engaged < ENGAGE_T_BP[-1]

  @property
  def t_since_engage(self) -> float | None:
    """seconds since engaging while the engage brake schedule still applies, else None"""
    t = 0.0 if self.t_engaged is None else self.t_engaged
    return t if t < ENGAGE_BRAKE_T_BP[-1] else None

  def up_step(self, active: bool) -> float:
    if not active:
      self.t_engaged = None
      return self.stock_up_jerk * self.dt
    if self.t_engaged is None:
      self.t_engaged = 0.0
    else:
      self.t_engaged += self.dt
    if self.t_engaged >= ENGAGE_T_BP[-1]:
      return self.stock_up_jerk * self.dt
    j_up = float(np.interp(self.t_engaged, ENGAGE_T_BP, ENGAGE_J_UP))
    return min(j_up, self.stock_up_jerk) * self.dt