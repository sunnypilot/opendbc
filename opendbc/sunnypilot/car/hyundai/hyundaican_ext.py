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
class HyundaiCanFDEXTParams(HyundaiCanEXTParams):
  targetDistance: float = 0.0
  lanelineLeft: float = 0.0
  lanelineRight: float = 0.0


class HyundaiCanEXT:
  # Hysteresis parameters
  LEAD_VISIBLE_HYSTERESIS_ON_FRAMES: int = 50
  LEAD_VISIBLE_HYSTERESIS_OFF_FRAMES: int = 50
  OBJECT_GAP_HYSTERESIS_FRAMES: int = 50
  LANE_POSITION_HYSTERESIS_FRAMES: int = 5

  def __init__(self):
    self.hyundaican_ext = HyundaiCanEXTParams()
    self.hyundaicanfd_ext = HyundaiCanFDEXTParams()
    self.lead_on_counter = 0
    self.lead_off_counter = 0
    self.lead_visible = False
    self.gap_counter = 0
    self.object_gap = 0
    self.lane_frame_counter = 0
    self.left_laneline = 15.0
    self.right_laneline = 15.0

  @staticmethod
  def _calculate_stopping_distance(vEgo: float, distance_setting: int) -> float:
      """Calculate safe distance to stop based on current speed and distance setting.

      Args:
          vEgo: Current speed in meters per second
          distance_setting: Distance setting (1-3)

      Returns:
          Safe stopping distance in meters
      """
      distance_setting_ttc_vals = {
          1: 1.2,
          2: 1.7,
          3: 2.0
      }

      # Default to shortest distance if invalid setting provided
      ttc_distance = distance_setting_ttc_vals.get(distance_setting, 1.2)

      # Calculate distance = speed * time
      return max(10., vEgo * ttc_distance)

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

  def hyundaican(self, CC_SP: structs.CarControlSP, hyundaiCanParams: HyundaiCanEXTParams) -> HyundaiCanEXTParams:
    lead_distance = CC_SP.leadDistance
    lead_rel_speed = CC_SP.leadRelSpeed
    objectRelGap = 0 if lead_distance == 0 else 2 if lead_rel_speed < -0.2 else 1

    self._update_lead_visible_hysteresis(CC_SP.leadVisible)
    self.object_gap, self.gap_counter = (
      HyundaiCanEXT._hysteresis_update(self.object_gap,
                                       HyundaiCanEXT._calculate_object_gap(lead_distance),
                                       self.gap_counter, self.OBJECT_GAP_HYSTERESIS_FRAMES))

    hyundaiCanParams.objectGap = self.object_gap
    hyundaiCanParams.objectRelGap = objectRelGap
    hyundaiCanParams.leadDistance = lead_distance
    hyundaiCanParams.leadRelSpeed = lead_rel_speed
    hyundaiCanParams.leadVisible = self.lead_visible

    return hyundaiCanParams

  def hyundaicanfd (self, CC_SP: structs.CarControlSP, CC: structs.CarControl, CS: structs.CarState) -> HyundaiCanFDEXTParams:

    self.hyundaican(CC_SP, self.hyundaicanfd_ext)

    lead_distance = CC_SP.leadDistance
    lead_rel_speed = CC_SP.leadRelSpeed
    objectRelGap = 0 if not self.lead_visible else 2 if lead_rel_speed < 0 else 1
    stopping_distance = HyundaiCanEXT._calculate_stopping_distance(CS.out.vEgo, CC.hudControl.leadDistanceBars)
    self.hyundaicanfd_ext.objectRelGap = objectRelGap
    self.hyundaicanfd_ext.targetDistance = stopping_distance if lead_distance == 0 else min (stopping_distance, lead_distance)
    self.hyundaicanfd_ext.lanelineLeft, self.hyundaicanfd_ext.lanelineRight = self._calculate_lane_positions(CS)

    return self.hyundaicanfd_ext

  def _calculate_lane_positions(self, CS: structs.CarState) -> tuple[float, float]:
    # Apply hysteresis
    self.lane_frame_counter += 1
    if self.lane_frame_counter >= self.LANE_POSITION_HYSTERESIS_FRAMES:

      leftlaneraw, rightlaneraw = CS.leftLanePosition, CS.rightLanePosition
      leftlanequal, rightlanequal = CS.leftLaneQuality, CS.rightLaneQuality

      scale_per_m = 15 / 1.7
      leftlane = abs(int(round(15 + (leftlaneraw - 1.7) * scale_per_m)))
      rightlane = abs(int(round(15 + (rightlaneraw - 1.7) * scale_per_m)))

      if leftlanequal not in (2, 3):
        leftlane = 0
      if rightlanequal not in (2, 3):
        rightlane = 0

      if leftlaneraw == -2.0248375:
        leftlane = 30 - rightlane
      if rightlaneraw == 2.0248375:
        rightlane = 30 - leftlane

      if leftlaneraw == rightlaneraw == 0:
        leftlane = rightlane = 15
      elif leftlaneraw == 0:
        leftlane = 30 - rightlane
      elif rightlaneraw == 0:
        rightlane = 30 - leftlane

      total = leftlane + rightlane
      if total == 0:
        leftlane = rightlane = 15
      else:
        leftlane = round((leftlane / total) * 30)
        rightlane = 30 - leftlane

      self.left_laneline = leftlane
      self.right_laneline = rightlane
      self.lane_frame_counter = 0

    return self.left_laneline, self.right_laneline

  def update(self, CC_SP: structs.CarControlSP, CC: structs.CarControl, CS: structs.CarState) -> None:
    self.hyundaican(CC_SP, self.hyundaican_ext)
    self.hyundaicanfd(CC_SP, CC, CS)
