"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs, rate_limit
from opendbc.sunnypilot.car.hyundai.longitudinal.config import CarTuningConfig, TUNING_CONFIGS, CAR_SPECIFIC_CONFIGS

JERK_THRESHOLD = 0.1
JERK_STEP = 0.1


def get_car_config(CP: structs.CarParams) -> CarTuningConfig:
  # Get car type flags from specific configs or determine from car flags
  car_config = CAR_SPECIFIC_CONFIGS.get(CP.carFingerprint)
  # If car is not in specific configs, determine from flags
  if car_config is None:
    car_config = TUNING_CONFIGS["DEFAULT"]

  return car_config


def get_longitudinal_tune(CP: structs.CarParams) -> None:
  config = get_car_config(CP)
  CP.longitudinalActuatorDelay = config.longitudinal_actuator_delay


def jerk_limited_integrator(desired_accel, last_accel, jerk_upper, jerk_lower, dt) -> float:
  if desired_accel >= last_accel:
    val = jerk_upper * dt
  else:
    val = jerk_lower * dt

  return rate_limit(desired_accel, last_accel, -val, val)
