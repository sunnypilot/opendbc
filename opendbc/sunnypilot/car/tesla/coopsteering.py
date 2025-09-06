"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections import namedtuple

from opendbc.car import structs
from opendbc.sunnypilot.car import get_param
from opendbc.car.carlog import carlog

CoopSteeringDataSP = namedtuple("CoopSteeringDataSP",
                                ["control_type"])


class CoopSteeringCarController:
  def __init__(self):
    super().__init__()
    self.coop_steering = CoopSteeringDataSP(False)

  @staticmethod
  def coop_steering_status_update(CC: structs.CarControl, CC_SP: structs.CarControlSP) -> CoopSteeringDataSP:
    coop_steering = get_param(CC_SP.params, "TeslaCoopSteering", "0") == "1"

    # debug output everything in CC_SP.params which is a list of structs.CarControlSP.Param
    for param in CC_SP.params:
      carlog.info("[COOP STEERING] CC_SP.params[%s]: %s", param.key, param.value, type(param.value))

    carlog.info("[COOP STEERING] coop_steering: %s, get_param: %s", coop_steering, get_param(CC_SP.params, "TeslaCoopSteering", "fallback"))
    control_type = 2 if coop_steering else 1

    return CoopSteeringDataSP(control_type)

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    self.coop_steering = self.coop_steering_status_update(CC, CC_SP)
