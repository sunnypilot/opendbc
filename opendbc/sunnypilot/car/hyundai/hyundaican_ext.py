"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass
from opendbc.car import structs


@dataclass
class HyundaiCanEXTParams:
  objectGap: int = 0
  objectRelGap: int = 0
  leadDistance: float = 0.0
  leadRelSpeed: float = 0.0

@dataclass
class HyundaiCanFDEXTParams:
  objectStatus: int = 0
  leadDistance: float = 0.0
  leadRelSpeed: float = 0.0


class HyundaiCanEXT:
  def __init__(self):
    self.hyundaican_ext = HyundaiCanEXTParams()
    self.hyundaicanfd_ext = HyundaiCanFDEXTParams()

  def hyundaican(self, CC_SP: structs.CarControlSP) -> HyundaiCanEXTParams:
    lead_distance = CC_SP.leadDistance
    lead_rel_speed = CC_SP.leadRelSpeed

    # Check if lead_distance is None before comparison
    if lead_distance is None or lead_distance == 0:
      objectGap = 0
    elif lead_distance < 20:
      objectGap = 2
    elif lead_distance < 25:
      objectGap = 3
    elif lead_distance < 30:
      objectGap = 4
    else:
      objectGap = 5

    objectRelGap = 0 if objectGap == 0 else 2 if lead_rel_speed < -0.2 else 1

    # Populate the external object
    self.hyundaican_ext.objectGap = objectGap
    self.hyundaican_ext.objectRelGap = objectRelGap
    self.hyundaican_ext.leadDistance = lead_distance
    self.hyundaican_ext.leadRelSpeed = lead_rel_speed

    return self.hyundaican_ext

  def hyundaicanfd (self, CC_SP: structs.CarControlSP) -> HyundaiCanFDEXTParams:
    lead_distance = CC_SP.leadDistance
    lead_rel_speed = CC_SP.leadRelSpeed

    # Check if lead_distance is None before comparison
    if lead_distance is None or lead_distance == 0:
      objectStatus = 0
    elif lead_distance < 20:
      objectStatus = 2
    elif lead_distance < 25:
      objectStatus = 3
    elif lead_distance < 30:
      objectStatus = 4
    else:
      objectStatus = 5

    # Populate the external object
    self.hyundaicanfd_ext.objectStatus = objectStatus
    self.hyundaicanfd_ext.leadDistance = lead_distance
    self.hyundaicanfd_ext.leadRelSpeed = lead_rel_speed

    return self.hyundaicanfd_ext

  def update (self, CC_SP: structs.CarControlSP) -> None:
    self.hyundaican(CC_SP)
    self.hyundaicanfd(CC_SP)
