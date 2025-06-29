"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.car import structs, DT_CTRL, rate_limit
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.longitudinal.config import CarTuningConfig, TUNING_CONFIGS, CAR_SPECIFIC_CONFIGS

JERK_THRESHOLD = 0.1
JERK_STEP = 0.1


class LongitudinalTuningType:
  OFF = 0
  DYNAMIC = 1
  PREDICTIVE = 2


def parse_float_list(param_str: str) -> list[float]:
  """Parse comma-separated string to list of floats."""
  try:
    return [float(x.strip()) for x in param_str.split(',')]
  except (ValueError, AttributeError):
    return []


def create_config_from_params(params_dict: dict[str, str], base_config: CarTuningConfig) -> CarTuningConfig:
  """Create a CarTuningConfig from parameter values."""
  try:
    config = CarTuningConfig(
      v_ego_stopping=float(params_dict.get("LongTuningVEgoStopping", str(base_config.v_ego_stopping))),
      v_ego_starting=float(params_dict.get("LongTuningVEgoStarting", str(base_config.v_ego_starting))),
      stopping_decel_rate=float(params_dict.get("LongTuningStoppingDecelRate", str(base_config.stopping_decel_rate))),
      lookahead_jerk_bp=parse_float_list(params_dict.get("LongTuningLookaheadJerkBp", ",".join(map(str, base_config.lookahead_jerk_bp)))),
      lookahead_jerk_upper_v=parse_float_list(params_dict.get("LongTuningLookaheadJerkUpperV", ",".join(map(str, base_config.lookahead_jerk_upper_v)))),
      lookahead_jerk_lower_v=parse_float_list(params_dict.get("LongTuningLookaheadJerkLowerV", ",".join(map(str, base_config.lookahead_jerk_lower_v)))),
      longitudinal_actuator_delay=float(params_dict.get("LongTuningLongitudinalActuatorDelay", str(base_config.longitudinal_actuator_delay))),
      jerk_limits=float(params_dict.get("LongTuningJerkLimits", str(base_config.jerk_limits))),
      upper_jerk_v=parse_float_list(params_dict.get("LongTuningUpperJerkV", ",".join(map(str, base_config.upper_jerk_v)))),
      lower_jerk_v=parse_float_list(params_dict.get("LongTuningLowerJerkV", ",".join(map(str, base_config.lower_jerk_v)))),
      min_upper_jerk=float(params_dict.get("LongTuningMinUpperJerk", str(base_config.min_upper_jerk))),
      min_lower_jerk=float(params_dict.get("LongTuningMinLowerJerk", str(base_config.min_lower_jerk))),
    )
    return config
  except (ValueError, TypeError):
    return base_config


def get_car_config(CP: structs.CarParams, params_dict: dict[str, str] = None) -> CarTuningConfig:
  # Check if custom tuning is enabled first
  custom_toggle_enabled = False
  if params_dict:
    custom_toggle_enabled = int(params_dict.get("LongTuningCustomToggle", "0")) == 1

  base_config = CAR_SPECIFIC_CONFIGS.get(CP.carFingerprint)
  # If car is not in specific configs, determine from flags
  if base_config is None:
    if CP.flags & HyundaiFlags.CANFD:
      base_config = TUNING_CONFIGS["CANFD"]
    elif CP.flags & HyundaiFlags.EV:
      base_config = TUNING_CONFIGS["EV"]
    elif CP.flags & HyundaiFlags.HYBRID:
      base_config = TUNING_CONFIGS["HYBRID"]
    else:
      base_config = TUNING_CONFIGS["DEFAULT"]

  # Custom Params
  if params_dict and custom_toggle_enabled:
    return create_config_from_params(params_dict, base_config)

  return base_config


def get_longitudinal_tune(CP: structs.CarParams, params_dict: dict[str, str] = None) -> None:
  config = get_car_config(CP, params_dict)
  CP.vEgoStopping = config.v_ego_stopping
  CP.vEgoStarting = config.v_ego_starting
  CP.stoppingDecelRate = config.stopping_decel_rate
  CP.startingState = False
  CP.longitudinalActuatorDelay = config.longitudinal_actuator_delay


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
