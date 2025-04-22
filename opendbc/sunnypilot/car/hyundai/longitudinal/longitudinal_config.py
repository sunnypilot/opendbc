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
  jerk_limits: tuple[float, float] = 0.25, 0.1
  longitudinal_actuator_delay: float = 0.45
  lower_jerk_multiplier: float = 1.0


# Default configurations for different car types
# Min jerk is set to 0.53 per (Horn et al., 2024)
TUNING_CONFIGS = {
  "CANFD": CarTuningConfig(
    stopping_decel_rate=0.275,
  ),
  "EV": CarTuningConfig(
    jerk_limits=(0.25, 0.1),
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
  ),
  CAR.KIA_NIRO_EV: CarTuningConfig(
    v_ego_stopping=0.1,
    stopping_decel_rate=0.1,
    jerk_limits=(0.25, 0.2),
    longitudinal_actuator_delay=0.45,
  ),
  CAR.HYUNDAI_IONIQ:CarTuningConfig(
    v_ego_stopping=0.25,
    stopping_decel_rate=0.4,
    jerk_limits=(0.25, 0.1),
    longitudinal_actuator_delay=0.45,
  )
}
