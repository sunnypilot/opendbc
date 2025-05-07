"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from dataclasses import dataclass

from opendbc.car import structs

from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_car_config

LongCtrlState = structs.CarControl.Actuators.LongControlState

GEN1_LOWER_JERK_BP = [-2.0, -1.5, -1.0, -0.25, -0.1, -0.025, -0.01, -0.005]
GEN1_LOWER_JERK_V  = [ 3.3,  2.5,  2.0,   1.9,  1.8,   1.65,  1.15,    0.5]


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
    self.accel_cmd = 0.0
    self.jerk_upper = 0.5
    self.jerk_lower = 0.5
    self.stopping = False
