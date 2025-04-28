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
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_car_config

LongCtrlState = structs.CarControl.Actuators.LongControlState

JERK_STEP = 0.1
JERK_THRESHOLD = 0.1
MIN_JERK = 0.5

GEN1_LOWER_JERK_BP = [-2.0, -1.5, -1.0, -0.25, -0.1, -0.025, -0.01, -0.005]
GEN1_LOWER_JERK_V  = [ 3.3,  2.5,  2.0,   1.9,  1.8,   1.65,  1.15,    0.5]


def jerk_limited_integrator(desired_accel, last_accel, jerk_upper, jerk_lower) -> float:
  if desired_accel >= last_accel:
    val = jerk_upper * DT_CTRL * 2
  else:
    val = jerk_lower * DT_CTRL * 2

  return rate_limit(desired_accel, last_accel, -val, val)


def ramp_update(current, target):
  error = target - current
  if abs(error) > JERK_THRESHOLD:
    return current + float(np.clip(error, -JERK_STEP, JERK_STEP))
  return target


@dataclass
class LongitudinalTuningState:
  accel_last: float = 0.0
  jerk: float = 0.0


class LongitudinalTuningController:
  """Longitudinal tuning methodology for HKG"""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP = CP
    self.CP_SP = CP_SP

    self.car_config = get_car_config(CP)
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

    velocity = CS.out.vEgo
    self.accel_cmd = CC.actuators.accel

    a_ego_blended = float(np.interp(velocity, [1.0, 2.0], [CS.aBasis, CS.out.aEgo]))
    self.aego.update(a_ego_blended)

    accel_error = self.accel_cmd - self.aego.x

    # Lookahead jerk: How much jerk is needed to reach desired accel in future_t seconds
    future_t_upper = float(np.interp(velocity, self.car_config.lookahead_jerk_bp, self.car_config.lookahead_jerk_upper_v))
    future_t_lower = float(np.interp(velocity, self.car_config.lookahead_jerk_bp, self.car_config.lookahead_jerk_lower_v))

    # Required jerk to reach target accel in lookahead window
    j_ego_upper = accel_error / future_t_upper
    j_ego_lower = accel_error / future_t_lower

    # Jerk is limited by the following conditions imposed by ISO 15622:2018.
    upper_speed_factor = 0.5
    if long_control_state == LongCtrlState.pid:
      upper_speed_factor = float(np.interp(velocity, [0.0, 5.0, 20.0], [2.0, 3.0, 1.6]))
    lower_speed_factor = float(np.interp(velocity, [0.0, 5.0, 20.0], [5.0, 5.0, 2.5]))

    lower_jerk = max(-j_ego_lower, MIN_JERK)
    if self.CP.radarUnavailable:
      lower_jerk = 5.0

    # Final jerk limits with thresholds
    desired_jerk_upper = min(max(j_ego_upper, MIN_JERK), upper_speed_factor)
    desired_jerk_lower = min(lower_jerk, lower_speed_factor)

    # Gen1 negative accel tune
    gen1_accel_error = a_ego_blended - self.state.accel_last
    if self.CP.radarUnavailable:
      gen1_lower_jerk = 5.0
    elif gen1_accel_error < 0:
      lower_max = self.car_config.jerk_limits
      original_value = np.array(GEN1_LOWER_JERK_V)
      dynamic_lower = original_value * (lower_max / original_value[0])
      gen1_lower_jerk = float(np.interp(gen1_accel_error, GEN1_LOWER_JERK_BP, dynamic_lower))
    else:
      gen1_lower_jerk = 0.5
    gen1_desired_jerk_lower = min(gen1_lower_jerk, lower_speed_factor)

    # New gen tune uses LONG_TUNING_BRAKING flag
    if self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING_BRAKING:
      self.jerk_upper = ramp_update(self.jerk_upper, desired_jerk_upper)
      self.jerk_lower = desired_jerk_lower
    else:
      self.jerk_upper = ramp_update(self.jerk_upper, desired_jerk_upper)
      self.jerk_lower = ramp_update(self.jerk_lower, gen1_desired_jerk_lower)
