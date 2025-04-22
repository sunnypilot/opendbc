"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass

from opendbc.car.hyundai.values import CAR


@dataclass
class CarTuningConfig:
  v_ego_stopping: float = 0.25
  v_ego_starting: float = 0.10
  stopping_decel_rate: float = 0.25
  lookahead: tuple[float, float, float] = 0.25, 0.1, 0.5  # (upper min, lower min , lower max)
  longitudinal_actuator_delay: float = 0.45
  jerk_limits: float = 3.3


# Default configurations for different car types
# Min jerk is set to 0.53 per (Horn et al., 2024)
TUNING_CONFIGS = {
  "CANFD": CarTuningConfig(
    stopping_decel_rate=0.275,
  ),
  "EV": CarTuningConfig(
    longitudinal_actuator_delay=0.45,
  ),
  "HYBRID": CarTuningConfig(
    v_ego_starting=0.12,
    stopping_decel_rate=0.30,
  ),
  "DEFAULT": CarTuningConfig()
}

# Car-specific configs
CAR_SPECIFIC_CONFIGS = {
  CAR.HYUNDAI_ELANTRA_2021: CarTuningConfig(
    stopping_decel_rate=0.8,
    lookahead=(0.25, 0.25, 0.5)
  ),
  CAR.KIA_NIRO_EV: CarTuningConfig(
    v_ego_stopping=0.1,
    stopping_decel_rate=0.1,
    lookahead=(0.25, 0.3, 0.6),
    jerk_limits=2.7,
    longitudinal_actuator_delay=0.45,
  ),
  CAR.HYUNDAI_IONIQ:CarTuningConfig(
    v_ego_stopping=0.25,
    stopping_decel_rate=0.4,
    lookahead=(0.25, 0.1, 0.5),
    jerk_limits=4.5,
    longitudinal_actuator_delay=0.45,
  )
}
