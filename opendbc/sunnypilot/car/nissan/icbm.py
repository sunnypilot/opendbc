"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase
from opendbc.car.nissan.nissancan import create_cruise_button_msg

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# Map generic send button states to CRUISE_THROTTLE button field names
BUTTON_FIELDS = {
  SendButtonState.increase: "RES_BUTTON",
  SendButtonState.decrease: "SET_BUTTON",
}



class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self._last_cruise_throttle_counter = None

  def update(self, CS, CC_SP, packer, frame, last_button_frame) -> list[CanData]:
    can_sends: list[CanData] = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    current_counter = CS.cruise_throttle_msg.get("COUNTER", 0)
    should_send_on_increase = (
      self._last_cruise_throttle_counter is not None and current_counter > self._last_cruise_throttle_counter
    )

    if self.ICBM.sendButton != SendButtonState.none and should_send_on_increase:
      send_field = BUTTON_FIELDS[self.ICBM.sendButton]
      # Keep the counter the same as car-provided; no modifications or offsets
      can_sends.append(create_cruise_button_msg(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, send_field))

    # Update stored counter only when it increases
    if self._last_cruise_throttle_counter is None or current_counter > self._last_cruise_throttle_counter:
      self._last_cruise_throttle_counter = current_counter

    return can_sends
