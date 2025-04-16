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
      delay_met = self.stopping_count > 1 / (DT_CTRL * 2)  # 1 second
      self.stopping = delay_met or strong_decel_request


  def calculate_jerk_and_accel(self, CC: structs.CarControl, CS: CarStateBase) -> None:

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
    a_ego_future = a_ego_blended + j_ego * future_t  # noqa: F841

    jerk_cmd = (accel_cmd - self.prev_accel) / (DT_CTRL * 2)  # noqa: F841

    # Separate into upper/lower
    self.jerk_upper = max(j_ego, 0.0)
    self.jerk_lower = abs(min(j_ego, 0.0))

    if self.stopping:
      self.desired_accel = 0.0
    else:
      self.desired_accel = float(np.clip(accel_cmd, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    self.actual_accel = jerk_limited_integrator(self.desired_accel, self.state.accel_last, self.jerk_upper, self.jerk_lower)
    self.state.accel_last = self.actual_accel
