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

    if self.ICBM.sendButton != SendButtonState.none:
      send_field = BUTTON_FIELDS[self.ICBM.sendButton]

      # Every 0.3s, continuously send for 0.1s. Sending only once per interval does not work on Nissan.
      # Every 0.2s was too fast.
      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.1:
        can_sends.append(create_cruise_button_msg(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, send_field))
        if (self.frame - self.last_button_frame) * DT_CTRL >= 0.3:
          self.last_button_frame = self.frame

    return can_sends
