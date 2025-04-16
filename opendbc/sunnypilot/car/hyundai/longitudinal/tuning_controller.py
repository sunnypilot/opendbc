"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import math
import numpy as np
from dataclasses import dataclass

from opendbc.car import structs, DT_CTRL, rate_limit, ACCELERATION_DUE_TO_GRAVITY
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase

from opendbc.car.hyundai.values import CarControllerParams, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_car_config
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState

ACCEL_WINDUP_LIMIT = 5.0 * DT_CTRL * 2  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -5.0 * DT_CTRL * 2  # m/s^2 / frame
JERK_STEP = 0.1
JERK_THRESHOLD = 0.1


def jerk_limited_integrator(desired_accel, last_accel, jerk_upper, jerk_lower) -> float:
  if desired_accel >= last_accel:
    val = jerk_upper * DT_CTRL * 2
  else:
    val = jerk_lower * DT_CTRL * 2

  return rate_limit(desired_accel, last_accel, -val, val)


def ramp_update(current, target):
  if abs(target - current) > JERK_THRESHOLD:
    return current + float(np.clip(target - current, -JERK_STEP, JERK_STEP))
  return current


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

  def get_stopping_state(self, long_control_state: LongCtrlState) -> None:
    long_control_stopping = long_control_state == LongCtrlState.stopping

    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING:
      self.stopping = long_control_stopping
      self.stopping_count = 0
      return

    if not long_control_stopping:
      self.stopping = False
      self.stopping_count = 0
      return

    if self.stopping_count > 1 / (DT_CTRL * 2):  # 1 second
      self.stopping = True

    self.stopping_count += 1

  def make_jerk(self, CC: structs.CarControl, CS: CarStateBase, long_control_state: LongCtrlState) -> None:
    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING_BRAKING:
      jerk_limit = 3.0 if long_control_state == LongCtrlState.pid else 1.0

      self.jerk_upper = jerk_limit
      self.jerk_lower = 5.0
      return

    # Apply acceleration filter
    self.desired_accel = CC.actuators.accel
    prev_filtered_accel = self.accel_filter.x
    self.accel_filter.update(self.desired_accel)
    filtered_accel = self.accel_filter.x

    # Calculate jerk
    self.state.jerk = (filtered_accel - prev_filtered_accel) / (DT_CTRL * 2)

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
    min_lower_jerk = self.car_config.jerk_limits[0] if (self.desired_accel - self.actual_accel) <= -0.01 else 0.5
    multiplier = self.car_config.lower_jerk_multiplier

    desired_jerk_upper = min(max(min_upper_jerk, self.state.jerk), accel_jerk_max)
    desired_jerk_lower = min(max(min_lower_jerk, -self.state.jerk * multiplier), decel_jerk_max)

    self.jerk_upper = ramp_update(self.jerk_upper, desired_jerk_upper)
    self.jerk_lower = ramp_update(self.jerk_lower, desired_jerk_lower)

  def calculate_a_value(self, CC: structs.CarControl) -> None:
    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING:
      self.desired_accel = CC.actuators.accel
      self.actual_accel = CC.actuators.accel
      return

    if not CC.longActive:
      self.desired_accel = 0.0
      self.actual_accel = 0.0
      self.state.accel_last = 0.0
      return

    # Force zero aReqRaw during StopReq
    if self.stopping:
      self.desired_accel = 0.0
    else:
      self.desired_accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    self.actual_accel = jerk_limited_integrator(self.desired_accel, self.state.accel_last, self.jerk_upper, self.jerk_lower)
    self.state.accel_last = self.actual_accel

  def calculate_jerk_and_accel(self, CC: structs.CarControl, CS: CarStateBase, pitch) -> None:
    long_control_state = CC.actuators.longControlState
    stopping = long_control_state == LongCtrlState.stopping
    has_lead = CC.hudControl.leadVisible

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

    # Calculate amount of acceleration PCM should apply to reach target, given pitch.
    # Clipped to only include downhill angles, avoids erroneously unsetting StopReq when on uphills
    accel_due_to_pitch = math.sin(min(pitch.x, 0.0)) * ACCELERATION_DUE_TO_GRAVITY

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
    accel_error_significant = abs(accel_error) >= 0.5
    jerk = abs(accel_error / (DT_CTRL * 2))
    if not has_lead and not accel_error_significant:
      # No lead car and accel error is not significant - use base value of 0.5
      desired_jerk_upper = 0.5
      desired_jerk_lower = 0.5
    else:
      # Lead car visible or accel error is significant - use original calculations
      if accel_error >= 0:
        # Acceleration is increasing - use upper jerk limit
        desired_jerk_upper = min(max(min_upper_jerk, jerk), accel_jerk_max)
        desired_jerk_lower = 0.5
      else:
        # Acceleration is decreasing - use lower jerk limit
        desired_jerk_upper = 0.5
        desired_jerk_lower = min(max(min_lower_jerk, jerk * multiplier), decel_jerk_max)

    self.jerk_upper = ramp_update(self.jerk_upper, desired_jerk_upper)
    self.jerk_lower = ramp_update(self.jerk_lower, desired_jerk_lower)

    net_acceleration_request = accel_cmd + accel_due_to_pitch
    net_acceleration_request_min = min(CC.actuators.accel + accel_due_to_pitch, net_acceleration_request)

    if net_acceleration_request_min < -0.5 and stopping:
      self.stopping = True
    elif not stopping:
      self.stopping = False

    if self.stopping:
      self.desired_accel = 0.0
    else:
      self.desired_accel = float(np.clip(accel_cmd, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    self.actual_accel = jerk_limited_integrator(self.desired_accel, self.state.accel_last, self.jerk_upper, self.jerk_lower)
    self.state.accel_last = self.actual_accel
