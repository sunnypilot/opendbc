"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from dataclasses import dataclass

from opendbc.car import structs, DT_CTRL, rate_limit
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase

from opendbc.car.hyundai.values import CarControllerParams, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_car_config
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState

ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL * 2  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -4.0 * DT_CTRL * 2  # m/s^2 / frame
JERK_STEP = 0.1
JERK_THRESHOLD = 0.1

def ramp_update(current, target):
  if abs(target - current) > JERK_THRESHOLD:
    return current + float(np.clip(target - current, -JERK_STEP, JERK_STEP))
  return current

def jerk_limited_integrator(desired_accel, last_accel, jerk_upper, jerk_lower) -> float:
  if desired_accel >= last_accel:
    val = jerk_upper * DT_CTRL * 2
  else:
    val = jerk_lower * DT_CTRL * 2

  return rate_limit(desired_accel, last_accel, -val, val)


@dataclass
class LongitudinalTuningState:
  accel_last: float = 0.0
  jerk: float = 0.0


class LongitudinalTuningController:
  """Longitudinal tuning methodology for HKG"""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP = CP
    self.CP_SP = CP_SP

    self.state = LongitudinalTuningState()
    self.car_config = get_car_config(CP)
    self.accel_filter = FirstOrderFilter(0.0, 0.25, DT_CTRL * 2)
    self.desired_accel = 0.0
    self.actual_accel = 0.0
    self.jerk_upper = 0.5
    self.jerk_lower = 0.5
    self.stopping = False
    self.stopping_count = 0

    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL * 2)
    self.prev_accel = 0.0

  def get_stopping_state(self, long_control_state: LongCtrlState, CC: structs.CarControl) -> None:
    long_control_stopping = long_control_state == LongCtrlState.stopping
    strong_decel_request = CC.longActive and CC.actuators.accel < -0.5

    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING:
      # Without LONG_TUNING, just follow the state
      self.stopping = long_control_stopping
      self.stopping_count = 0
      return

    if not long_control_stopping:
      # If state is not stopping, only stop if strong deceleration override is active
      self.stopping = strong_decel_request
      self.stopping_count = 0
    else:
      # Apply delay logic OR strong deceleration override
      self.stopping_count += 1
      delay_met = self.stopping_count > 1 / (DT_CTRL * 2)  # noqa: F841
      self.stopping = strong_decel_request


  def calculate_jerk_and_accel(self, CC: structs.CarControl, CS: CarStateBase) -> None:
    long_control_state = CC.actuators.longControlState

    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING:
      self.desired_accel = CC.actuators.accel
      self.actual_accel = CC.actuators.accel
      return

    if not CC.longActive:
      self.desired_accel = 0.0
      self.actual_accel = 0.0
      self.state.accel_last = 0.0
      self.aego.x = 0.0
      return

    accel_cmd = CC.actuators.accel

    # Internal PCM gas command can get stuck unwinding from negative accel so we apply a rate limit
    accel_cmd = rate_limit(accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, ACCEL_WINDUP_LIMIT)
    self.prev_accel = accel_cmd

    if self.CP.flags & HyundaiFlags.CANFD:
      # TODO-SP: use it if found in DBC
      a_ego_blended = CS.out.aEgo
    else:
      a_ego_blended = float(np.interp(CS.out.vEgo, [1.0, 2.0], [CS.aBasis, CS.out.aEgo]))

    prev_aego = self.aego.x
    self.aego.update(a_ego_blended)
    j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 2)

    future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.25, 0.5]))
    a_ego_future = a_ego_blended + j_ego * future_t

    # Jerk is limited by the following conditions imposed by ISO 15622:2018
    velocity = CS.out.vEgo
    if velocity < 5.0:
      decel_jerk_max = self.car_config.jerk_limits[1]
    elif velocity > 20.0:
      decel_jerk_max = 2.5
    else:   # Between 5 m/s and 20 m/s
      decel_jerk_max = 5.83 - (velocity/6)

    accel_jerk_max = self.car_config.jerk_limits[2] if long_control_state == LongCtrlState.pid else 1.0
    min_upper_jerk = 0.5 if (velocity > 3.0) else 0.725
    min_lower_jerk = self.car_config.jerk_limits[0] if (accel_cmd - a_ego_future) <= -0.01 else 0.5
    multiplier = self.car_config.lower_jerk_multiplier

    # Calculate desired upper and lower jerk limits based on acceleration error
    accel_error = accel_cmd - a_ego_future
    jerk = abs(accel_error / (DT_CTRL * 2))
    if accel_error >= 0:
      # Acceleration is increasing - use upper jerk limit
      desired_jerk_upper = min(max(min_upper_jerk, jerk), accel_jerk_max)
      desired_jerk_lower = 0.5
    else:
      # Acceleration is decreasing - use lower jerk limit
      desired_jerk_upper = 0.5
      # For deceleration, scale jerk based on how negative the acceleration error is
      # This provides more gradual braking that doesn't suddenly become aggressive
      error_magnitude = abs(accel_error)

      # Softer jerk when already decelerating to prevent compounding braking effect
      if a_ego_blended < -0.1:
        # Progressive scaling of jerk for smoother deceleration
        # This reduces jerk when already braking to prevent aggressive braking cascade
        jerk_scale = float(np.clip(0.6 + (error_magnitude * 0.2), 0.6, 1.0))
        jerk_value = jerk * jerk_scale
      else:
        # Standard jerk calculation when not already decelerating
        jerk_value = jerk * multiplier

      # Progressive cap on maximum jerk to prevent sudden aggressive braking
      # This helps maintain a more consistent deceleration profile
      max_jerk = min(decel_jerk_max, 2.5 + min(error_magnitude * 0.3, 0.7))

      desired_jerk_lower = min(max(min_lower_jerk, jerk_value), max_jerk)

    self.jerk_upper = ramp_update(self.jerk_upper, desired_jerk_upper)
    self.jerk_lower = ramp_update(self.jerk_lower, desired_jerk_lower)

    if self.stopping:
      self.desired_accel = 0.0
    else:
      self.desired_accel = float(np.clip(accel_cmd, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    self.actual_accel = jerk_limited_integrator(self.desired_accel, self.state.accel_last, self.jerk_upper, self.jerk_lower)
    self.state.accel_last = self.actual_accel
