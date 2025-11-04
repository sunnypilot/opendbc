"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, downquark7, and a number of other contributors.
This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase
from opendbc.car.nissan import nissancan

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTONS = {
  SendButtonState.increase: "RES_BUTTON",
  SendButtonState.decrease: "SET_BUTTON",
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.queue_button = None
    self.send_button = None

  def update(self, CS, CC_SP, packer, frame, last_button_frame) -> list[CanData]:
    can_sends: list[CanData] = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    # may add more buttons to this handler later
    # always need to queue manual button presses if icbm isn't sending because this is how we keep track of the timer resets
    if CS.cruise_throttle_msg["FOLLOW_DISTANCE_BUTTON"]:
      self.queue_button = "FOLLOW_DISTANCE_BUTTON"

    if not self.queue_button and self.ICBM.sendButton != SendButtonState.none:
        self.queue_button = BUTTONS[self.ICBM.sendButton]

    if (self.frame - self.last_button_frame) * DT_CTRL < 0.07:
      can_sends.append(nissancan.create_cruise_throttle_msg(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, self.frame, "NO_BUTTON_PRESSED"))

    elif (self.frame - self.last_button_frame) * DT_CTRL <= 0.11:
      can_sends.append(nissancan.create_cruise_throttle_msg(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, self.frame, self.send_button))

    if (self.frame - self.last_button_frame) * DT_CTRL >= 0.11:
      manually_holding_button = (self.queue_button == self.send_button) and not CS.cruise_throttle_msg["NO_BUTTON_PRESSED"]
      if self.send_button and not manually_holding_button:
        self.last_button_frame = self.frame
      self.send_button = self.queue_button
      self.queue_button = None

    return can_sends
