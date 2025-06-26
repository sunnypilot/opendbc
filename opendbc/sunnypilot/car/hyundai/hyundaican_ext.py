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
  leadVisible: bool = False

@dataclass
class HyundaiCanFDEXTParams:
  objectGap: int = 0
  objectRelGap: int = 0
  leadDistance: float = 0.0
  leadRelSpeed: float = 0.0
  leadVisible: bool = False
  targetDistance: float = 0.0


class HyundaiCanEXT:
  # Hysteresis parameters
  LEAD_VISIBLE_HYSTERESIS_ON_FRAMES: int = 1
  LEAD_VISIBLE_HYSTERESIS_OFF_FRAMES: int = 10
  OBJECT_GAP_HYSTERESIS_FRAMES: int = 10

  def __init__(self):
    self.hyundaican_ext = HyundaiCanEXTParams()
    self.hyundaicanfd_ext = HyundaiCanFDEXTParams()
    self.lead_on_counter = 0
    self.lead_off_counter = 0
    self.lead_visible = False
    self.gap_counter = 0
    self.object_gap = 0

  @staticmethod
  def _calculate_safe_distance(vEgo: float, distance_setting: int) -> float:
    """Calculate safe distance to stop based on current speed and distance setting.

    Args:
        vEgo: Current speed in meters per second
        distance_setting: Distance setting (1-3)

    Returns:
        Safe stopping distance in meters
    """
    time_gaps = {
      1: 1.2,
      2: 1.7,
      3: 2
    }

    # Default to shortest distance if invalid setting provided
    time_gap = time_gaps.get(distance_setting, 1.2)

    # Calculate distance = speed * time
    return max(10, vEgo * time_gap)

  @staticmethod
  def _hysteresis_update(current, new_value, counter, threshold):
    if new_value == current:
      return current, 0
    counter += 1
    return (new_value, 0) if counter >= threshold else (current, counter)

  @staticmethod
  def _calculate_object_gap(lead_distance: float | None) -> int:
    if lead_distance is None or lead_distance == 0:
      return 0
    elif lead_distance < 20:
      return 2
    elif lead_distance < 25:
      return 3
    elif lead_distance < 30:
      return 4
    else:
      return 5

  def _update_lead_visible_hysteresis(self, raw_lead_visible: bool) -> None:
    if raw_lead_visible:
      self.lead_on_counter += 1
      self.lead_off_counter = 0
      if self.lead_on_counter >= self.LEAD_VISIBLE_HYSTERESIS_ON_FRAMES:
        self.lead_visible = True
    else:
      self.lead_off_counter += 1
      self.lead_on_counter = 0
      if self.lead_off_counter >= self.LEAD_VISIBLE_HYSTERESIS_OFF_FRAMES:
        self.lead_visible = False

  def hyundaican(self, CC_SP: structs.CarControlSP) -> HyundaiCanEXTParams:
    lead_distance = CC_SP.leadDistance
    lead_rel_speed = CC_SP.leadRelSpeed
    objectRelGap = 0 if lead_distance == 0 else 2 if lead_rel_speed < -0.2 else 1

    self._update_lead_visible_hysteresis(CC_SP.leadVisible)
    self.object_gap, self.gap_counter = (
      HyundaiCanEXT._hysteresis_update(self.object_gap,
                                       HyundaiCanEXT._calculate_object_gap(lead_distance),
                                       self.gap_counter, self.OBJECT_GAP_HYSTERESIS_FRAMES))

    self.hyundaican_ext.objectGap = self.object_gap
    self.hyundaican_ext.objectRelGap = objectRelGap
    self.hyundaican_ext.leadDistance = lead_distance
    self.hyundaican_ext.leadRelSpeed = lead_rel_speed
    self.hyundaican_ext.leadVisible = self.lead_visible

    return self.hyundaican_ext

  def hyundaicanfd (self, CC_SP: structs.CarControlSP, CC: structs.CarControl, CS: structs.CarState) -> HyundaiCanFDEXTParams:
    lead_distance = CC_SP.leadDistance
    lead_rel_speed = CC_SP.leadRelSpeed
    objectRelGap = 0 if not self.lead_visible else 2 if lead_rel_speed < 0 else 1
    safe_distance = HyundaiCanEXT._calculate_safe_distance(CS.out.vEgo, CC.hudControl.leadDistanceBars)

    self._update_lead_visible_hysteresis(CC_SP.leadVisible)
    self.object_gap, self.gap_counter = (
      HyundaiCanEXT._hysteresis_update(self.object_gap,
                                       HyundaiCanEXT._calculate_object_gap(lead_distance),
                                       self.gap_counter, self.OBJECT_GAP_HYSTERESIS_FRAMES))

    self.hyundaicanfd_ext.objectGap = self.object_gap
    self.hyundaicanfd_ext.objectRelGap = objectRelGap
    self.hyundaicanfd_ext.leadDistance = lead_distance
    self.hyundaicanfd_ext.leadRelSpeed = lead_rel_speed
    self.hyundaicanfd_ext.leadVisible = self.lead_visible
    self.hyundaicanfd_ext.targetDistance = safe_distance if lead_distance == 0 else min (safe_distance, lead_distance)

    return self.hyundaicanfd_ext

  def update(self, CC_SP: structs.CarControlSP, CC: structs.CarControl, CS: structs.CarState) -> None:
    self.hyundaican(CC_SP)
    self.hyundaicanfd(CC_SP, CC, CS)
