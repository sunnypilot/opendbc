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

      # Do it like Mazda: every >0.2s window, send 3 messages with specific counter offsets [1, 1, 0], then skip once
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        self.button_frame += 1
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if button_counter_offset is not None:
          # compute the counter similarly to Mazda's crz_btns_counter behavior (assume 2-bit counter on Nissan)
          base_counter = CS.cruise_throttle_msg.get("COUNTER", 0)
          counter = (base_counter + button_counter_offset) % 4
          can_sends.append(create_cruise_button_msg(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, send_field, counter))
          self.last_button_frame = self.frame

    return can_sends
