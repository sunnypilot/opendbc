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

  def get_stopping_state(self, long_control_state: LongCtrlState, CC: structs.CarControl, CS: CarStateBase) -> None:
    long_control_stopping = long_control_state == LongCtrlState.stopping
    strong_decel_request = CC.longActive and CC.actuators.accel < -0.5

    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING:
      # Without LONG_TUNING, just follow the state
      self.stopping = long_control_stopping
      self.stopping_count = 0
      return

    if not long_control_stopping and CS.out.vEgo < 5.0:
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

    # Jerk is limited by the following conditions imposed by ISO 15622:2018
    velocity = CS.out.vEgo
    speed_factor = float(np.interp(velocity, [0.0, 5.0, 20.0], [5.0, 5.0, 2.5]))

    accel_error = a_ego_blended - self.state.accel_last
    if accel_error <= -0.01:
      # Interpolate min_lower_jerk from 1.0 at -0.01 to 5.0 at -3.5
      lower_jerk = float(np.interp(accel_error, [-0.01, -0.2, -0.5, -3.5], [1.0, 2.5, 3.3, 5.0]))
    else:
      lower_jerk = 0.5

    upper_jerk = 0.5 if (velocity > 3.0) else 0.725
    accel_jerk_max = self.car_config.jerk_limits[2] if long_control_state == LongCtrlState.pid else 1.0

    desired_jerk_upper = min(max(upper_jerk, j_ego), accel_jerk_max)
    desired_jerk_lower = min(lower_jerk, speed_factor)

    self.jerk_upper = ramp_update(self.jerk_upper, desired_jerk_upper)
    self.jerk_lower = ramp_update(self.jerk_lower, desired_jerk_lower)

    if self.stopping:
      self.desired_accel = 0.0
    else:
      self.desired_accel = float(np.clip(accel_cmd, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    self.actual_accel = jerk_limited_integrator(self.desired_accel, self.state.accel_last, self.jerk_upper, self.jerk_lower)
    self.state.accel_last = self.actual_accel
