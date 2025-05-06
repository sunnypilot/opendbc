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
  objectStatus: int = 0
  leadDistance: float = 0.0
  leadRelSpeed: float = 0.0
  leadVisible: bool = False


class HyundaiCanEXT:
  # Hysteresis parameters for lead visibility
  # Turns on if lead is present for ON_FRAMES
  LEAD_VISIBLE_HYSTERESIS_ON_FRAMES: int = 1
  # Turns off if lead is absent for OFF_FRAMES
  LEAD_VISIBLE_HYSTERESIS_OFF_FRAMES: int = 5 # e.g., 0.1 sec at 50Hz

  def __init__(self):
    self.hyundaican_ext = HyundaiCanEXTParams()
    self.hyundaicanfd_ext = HyundaiCanFDEXTParams()
    self.lead_visible_on_counter: int = 0
    self.lead_visible_off_counter: int = 0
    self.global_hysteretic_lead_visible: bool = False

  def _update_global_hysteretic_lead_state(self, raw_lead_present: bool) -> None:
    """ Updates the global hysteretic lead state based on raw_lead_present and internal counters. """
    if raw_lead_present:
      self.lead_visible_on_counter = min(self.lead_visible_on_counter + 1, self.LEAD_VISIBLE_HYSTERESIS_ON_FRAMES)
      self.lead_visible_off_counter = 0
    else:
      self.lead_visible_off_counter = min(self.lead_visible_off_counter + 1, self.LEAD_VISIBLE_HYSTERESIS_OFF_FRAMES)
      self.lead_visible_on_counter = 0

    if self.lead_visible_on_counter >= self.LEAD_VISIBLE_HYSTERESIS_ON_FRAMES:
      self.global_hysteretic_lead_visible = True
    elif self.lead_visible_off_counter >= self.LEAD_VISIBLE_HYSTERESIS_OFF_FRAMES:
      self.global_hysteretic_lead_visible = False
    # If neither counter has reached its threshold, global_hysteretic_lead_visible retains its previous state.

  def _calculate_object_gap(self, lead_distance: float | None) -> int:
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

  def hyundaican(self, CC_SP: structs.CarControlSP) -> HyundaiCanEXTParams:
    lead_distance = CC_SP.leadDistance
    lead_rel_speed = CC_SP.leadRelSpeed

    objectGap = self._calculate_object_gap(lead_distance)
    objectRelGap = 0 if objectGap == 0 else 2 if lead_rel_speed < -0.2 else 1

    self.hyundaican_ext.objectGap = objectGap
    self.hyundaican_ext.objectRelGap = objectRelGap
    self.hyundaican_ext.leadDistance = lead_distance
    self.hyundaican_ext.leadRelSpeed = lead_rel_speed
    self.hyundaican_ext.leadVisible = self.global_hysteretic_lead_visible

    return self.hyundaican_ext

  def hyundaicanfd (self, CC_SP: structs.CarControlSP) -> HyundaiCanFDEXTParams:
    lead_distance = CC_SP.leadDistance
    lead_rel_speed = CC_SP.leadRelSpeed

    objectStatus = self._calculate_object_gap(lead_distance)

    self.hyundaicanfd_ext.objectStatus = objectStatus
    self.hyundaicanfd_ext.leadDistance = lead_distance
    self.hyundaicanfd_ext.leadRelSpeed = lead_rel_speed
    self.hyundaicanfd_ext.leadVisible = self.global_hysteretic_lead_visible

    return self.hyundaicanfd_ext

  def update (self, CC_SP: structs.CarControlSP) -> None:
    self._update_global_hysteretic_lead_state(CC_SP.leadVisible)
    self.hyundaican(CC_SP)
    self.hyundaicanfd(CC_SP)
