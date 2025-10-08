"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
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

  def update(self, CS, CC_SP, packer, frame, last_button_frame) -> list[CanData]:
    can_sends: list[CanData] = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    # Determine whether we should send a button or just a heartbeat (no button)
    send_field = None
    if self.ICBM.sendButton != SendButtonState.none:
      candidate_field = BUTTON_FIELDS[self.ICBM.sendButton]
      # Every 0.3s, continuously send for 0.1s. Sending only once per interval does not work on Nissan.
      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.1:
        send_field = candidate_field
        if (self.frame - self.last_button_frame) * DT_CTRL >= 0.3:
          self.last_button_frame = self.frame

    # Send CRUISE_THROTTLE every other frame with updated counter, even when no button is pressed
    if (self.frame % 2) == 0:
      counter = (self.frame // 2) % 4
      can_sends.append(create_cruise_button_msg(packer, self.CP.carFingerprint, counter, CS.cruise_throttle_msg, send_field))

    return can_sends
