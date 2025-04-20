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

from opendbc.car.hyundai.values import CarControllerParams
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState

JERK_STEP = 0.1
JERK_THRESHOLD = 0.1

UPPER_START_JERK_BP = [0.005,  0.6,  1.0,  1.6,  2.0]
UPPER_START_JERK_V  = [  0.5,   0.6,  1.0,    1.6,  2.0]

UPPER_JERK_BP = [0.005, 0.03, 0.5, 1.0,  1.5, 2.0]
UPPER_JERK_V  = [  0.5,  0.6, 1.0, 1.5, 1.25, 2.0]

LOWER_JERK_BP = [-2.2, -1.6, -1.0, -0.25, -0.1, -0.025, -0.01, -0.005]
LOWER_JERK_V  = [ 3.3,  2.4,  1.85,   1.7,  1.5,   1.25,   1.0,    0.5]


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
    self.desired_accel = 0.0
    self.actual_accel = 0.0
    self.accel_cmd = 0.0
    self.jerk_upper = 0.5
    self.jerk_lower = 0.5
    self.stopping = False
    self.stopping_count = 0

    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL * 2)

  def get_stopping_state(self, long_control_state: LongCtrlState, long_state_last: LongCtrlState) -> None:
    stopping = long_control_state == LongCtrlState.stopping

    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING:
      self.stopping = stopping
      self.stopping_count = 0
      return

    if not stopping:
      self.stopping = False
      self.stopping_count = 0
      return

    # when the last state was off
    if long_state_last == LongCtrlState.off:
      self.stopping = True
      return

    if self.stopping_count > 1 / (DT_CTRL * 2):
      self.stopping = True

    self.stopping_count += 1

  def calculate_a_value(self, CC: structs.CarControl) -> None:
    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING or self.CP.radarUnavailable:
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
      self.desired_accel = float(np.clip(self.accel_cmd, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    self.actual_accel = jerk_limited_integrator(self.desired_accel, self.state.accel_last, self.jerk_upper, self.jerk_lower)
    self.state.accel_last = self.actual_accel

    return

  def calculate_jerk(self, CC: structs.CarControl, CS: CarStateBase, long_control_state: LongCtrlState) -> None:
    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING:
      jerk_limit = 3.0 if long_control_state == LongCtrlState.pid else 1.0

      self.jerk_upper = jerk_limit
      self.jerk_lower = 5.0
      return

    self.accel_cmd = CC.actuators.accel

    a_ego_blended = float(np.interp(CS.out.vEgo, [1.0, 2.0], [CS.aBasis, CS.out.aEgo]))

    prev_aego = self.aego.x
    self.aego.update(a_ego_blended)
    j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 2)

    future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.1, 0.2]))
    a_ego_blended_ = a_ego_blended + j_ego * future_t   # noqa: F841

    accel_error_ = self.accel_cmd - a_ego_blended   # noqa: F841
    accel_error = a_ego_blended - self.state.accel_last

  # Jerk is limited by the following conditions imposed by ISO 15622:2018.
    velocity = CS.out.vEgo
    lower_speed_factor = float(np.interp(velocity, [0.0, 5.0, 20.0], [5.0, 5.0, 2.5]))
    upper_speed_factor = 1.0
    if long_control_state == LongCtrlState.pid:
      upper_speed_factor = float(np.interp(velocity, [0.0, 5.0, 20.0], [1.0, 2.2, 1.0]))

    if accel_error > 0.005:
      _upper_bp = UPPER_START_JERK_BP if velocity < 5.0 else UPPER_JERK_BP
      _upper_v = UPPER_START_JERK_V if velocity < 5.0 else UPPER_JERK_V
      upper_jerk = float(np.interp(accel_error, _upper_bp, _upper_v))
    else:
      upper_jerk = 0.5

    if self.CP.radarUnavailable:
      lower_jerk = 5.0
    elif accel_error < -0.005:
      lower_jerk = float(np.interp(accel_error, LOWER_JERK_BP, LOWER_JERK_V))
    else:
      lower_jerk = 0.5

    desired_jerk_upper = min(upper_jerk, upper_speed_factor)
    desired_jerk_lower = min(lower_jerk, lower_speed_factor)

    self.jerk_upper = (ramp_update(self.jerk_upper, desired_jerk_upper)
      if self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING_BRAKING else desired_jerk_upper)
    self.jerk_lower = (ramp_update(self.jerk_lower, desired_jerk_lower)
      if self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING_BRAKING else desired_jerk_lower)
