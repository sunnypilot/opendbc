"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from abc import ABC, abstractmethod
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags


class LeadData(ABC):
  def __init__(self, object_gap: int, lead_distance: float, lead_rel_speed: float, lead_visible: bool,
               target_distance: float, left_laneline: float = 0, right_laneline: float = 0):
    self.object_gap = object_gap
    self.lead_distance = lead_distance
    self.lead_rel_speed = lead_rel_speed
    self.lead_visible = lead_visible
    self.target_distance = target_distance
    self.left_laneline = left_laneline
    self.right_laneline = right_laneline

  @property
  @abstractmethod
  def object_rel_gap(self) -> int:
    raise NotImplementedError("Subclasses must implement this method")


class CanLeadData(LeadData):
  @property
  def object_rel_gap(self) -> int:
    return 0 if self.lead_distance == 0 else 2 if self.lead_rel_speed < -0.2 else 1


class CanFdLeadData(LeadData):
  @property
  def object_rel_gap(self) -> int:
    return 0 if not self.lead_visible else 2 if self.lead_rel_speed < 0 else 1


def _hysteresis_update(current, new_value, counter, threshold):
  """
  Updates a value based on a hysteresis threshold mechanism. This function
  compares a new value against the current value and uses a counter to detect
  when a transition should occur, avoiding rapid oscillations between states.
  A new value will only be adopted if it differs from the current value and
  the counter reaches the specified threshold.

  :param current: The current value being tracked.
  :param new_value: The potential new value to compare against the current.
  :param counter: The count of consecutive different values encountered.
  :param threshold: The minimum count required before switching to the new value.
  :return: A tuple containing:
           - The updated current value, which is either the original current
             value or the new value if the hysteresis condition was met.
           - The updated counter, reset to 0 if the new value was adopted,
             or incremented by 1 otherwise.
  """
  if new_value == current:
    return current, 0
  counter += 1
  return (new_value, 0) if counter >= threshold else (current, counter)


class LeadDataCarController:
  # Hysteresis parameters
  LEAD_HYSTERESIS_FRAMES: int = 50
  LANE_POSITION_HYSTERESIS_FRAMES: int = 5

  def __init__(self, CP: structs.CarParams):
    self.CP = CP

    self.lead_one = {}
    self.lead_two = {}

    self._lead_on_counter = 0
    self._lead_off_counter = 0
    self.lead_visible = False
    self.gap_counter = 0
    self.object_gap = 0
    self.lead_distance = 0
    self.lead_rel_speed = 0
    self.target_distance = 0
    self.lane_frame_counter = 0
    self.left_laneline = 15.0
    self.right_laneline = 15.0

  def _update_object_gap(self, lead_distance: float | None):
    new_gap = 5  # Default gap value if no lead distance is provided
    if lead_distance is None or lead_distance == 0:
      new_gap = 0
    elif lead_distance < 20:
      new_gap = 2
    elif lead_distance < 25:
      new_gap = 3
    elif lead_distance < 30:
      new_gap = 4

    self.object_gap, self.gap_counter = _hysteresis_update(self.object_gap, new_gap, self.gap_counter, self.LEAD_HYSTERESIS_FRAMES)

  def _update_lead_visible_hysteresis(self, raw_lead_visible: bool):
    counter = self._lead_on_counter if raw_lead_visible else self._lead_off_counter
    self.lead_visible, counter = _hysteresis_update(self.lead_visible, raw_lead_visible, counter, self.LEAD_HYSTERESIS_FRAMES)

    if raw_lead_visible:
      self._lead_on_counter = counter
      self._lead_off_counter = 0  # reset opposite counter
    else:
      self._lead_off_counter = counter
      self._lead_on_counter = 0  # reset opposite counter

  def _update_target_distance(self, vEgo: float, distance_setting: int):

    distance_setting_ttc_vals = {
      1: 1.2,
      2: 1.7,
      3: 2.0
    }

    ttc_distance = distance_setting_ttc_vals.get(distance_setting, distance_setting_ttc_vals[1])
    stopping_distance =  max(10., vEgo * ttc_distance)

    self.target_distance = stopping_distance if self.lead_distance == 0 else min (stopping_distance, self.lead_distance)

  def _update_lane_positioning(self, CS: structs.CarState):
    # Apply hysteresis
    # self.lane_frame_counter += 1
    # if self.lane_frame_counter >= self.LANE_POSITION_HYSTERESIS_FRAMES:

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
    # self.lane_frame_counter = 0

  def update(self, CC_SP: structs.CarControlSP, CC: structs.CarControl, CS: structs.CarState) -> None:
    self.lead_one = CC_SP.leadOne
    self.lead_two = CC_SP.leadTwo

    self.lead_distance = self.lead_one.dRel
    self.lead_rel_speed = self.lead_one.vRel
    self._update_lead_visible_hysteresis(self.lead_one.status)
    self._update_object_gap(self.lead_distance)
    self._update_target_distance(CS.out.vEgo, CC.hudControl.leadDistanceBars)
    self._update_lane_positioning(CS)

  @property
  def lead_data(self) -> CanLeadData | CanFdLeadData:
    if self.CP.flags & HyundaiFlags.CANFD:
      return CanFdLeadData(self.object_gap, self.lead_distance, self.lead_rel_speed, self.lead_visible,
                           self.target_distance, self.left_laneline, self.right_laneline)

    return CanLeadData(self.object_gap, self.lead_distance, self.lead_rel_speed, self.lead_visible,
                       self.target_distance)
