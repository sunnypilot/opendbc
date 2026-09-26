"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

ONSET_T_BP = [0.0, 0.2, 0.6, 1.0, 1.5]   # s
ONSET_J_DOWN = [0.25, 0.4, 0.8, 1.8, 4.0]  # m/s^3
HARD_BRAKE_ACCEL = -2.0  # m/s^2


class BrakeOnsetShaper:
  def __init__(self, dt: float, stock_down_jerk: float):
    self.dt = dt
    self.stock_down_jerk = stock_down_jerk
    self.t_onset = 0.0

  def reset(self) -> None:
    self.t_onset = 0.0

  def down_step(self, accel_request: float, prev_accel: float, bypass: bool = False) -> float:
    if bypass:
      self.t_onset = 0.0
      return -self.stock_down_jerk * self.dt

    gentlest_step = -ONSET_J_DOWN[0] * self.dt
    onset = (accel_request - prev_accel) < gentlest_step - 1e-9
    if onset:
      j_down = float(np.interp(self.t_onset, ONSET_T_BP, ONSET_J_DOWN))
      self.t_onset = min(self.t_onset + self.dt, ONSET_T_BP[-1])
    else:
      self.t_onset = max(self.t_onset - self.dt, 0.0)
      j_down = self.stock_down_jerk
    return -min(j_down, self.stock_down_jerk) * self.dt

  def is_urgent(self, accel_request: float, fcw: bool) -> bool:
    return fcw or accel_request < HARD_BRAKE_ACCEL
