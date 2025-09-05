"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car.carlog import carlog
from collections import namedtuple

from opendbc.car import structs

MadsDataSP = namedtuple("MadsDataSP",
                        ["steering_only"])


class MadsCarController:
  def __init__(self):
    super().__init__()
    self.mads = MadsDataSP(False)

  @staticmethod
  def mads_status_update(CC: structs.CarControl, CC_SP: structs.CarControlSP) -> MadsDataSP:
    steering_only = CC_SP.mads.available and not CC.enabled
    carlog.info(f"[TESLA MADS] mads.available={CC_SP.mads.available} CC.enabled={CC.enabled} steering_only={steering_only}")
    return MadsDataSP(steering_only)

  def update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    self.mads = self.mads_status_update(CC, CC_SP)
