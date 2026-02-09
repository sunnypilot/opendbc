"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, downquark7, and a number of other contributors.
This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase
from opendbc.car.nissan import nissancan

from collections import deque

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTONS = {
  SendButtonState.increase: "RES_BUTTON",
  SendButtonState.decrease: "SET_BUTTON",
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.__state = 0
    self.__queued_buttons = deque()
    self.__distance_button = False
    self.__prev_distance_button = False
    self.__set_button = False
    self.__prev_set_button = False
    self.__res_button = False
    self.__prev_res_button = False

  def update(self, CS, CC_SP, packer, frame, last_button_frame) -> list[CanData]:
    can_sends: list[CanData] = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    self.__prev_distance_button = self.__distance_button
    self.__distance_button = CS.cruise_throttle_msg["FOLLOW_DISTANCE_BUTTON"]

    self.__prev_set_button = self.__set_button
    self.__set_button = CS.cruise_throttle_msg["SET_BUTTON"]

    self.__prev_res_button = self.__res_button
    self.__res_button = CS.cruise_throttle_msg["RES_BUTTON"]

    # record if button press blocked during icbm send cycle to send it later for a consistent user experience
    # can add other buttons to this if necessary
    if self.__state in (1, 2):
      if self.__distance_button and not self.__prev_distance_button:
        self.__queued_buttons.append("FOLLOW_DISTANCE_BUTTON")

      if self.__set_button and not self.__prev_set_button:
        self.__queued_buttons.append("SET_BUTTON")

      if self.__res_button and not self.__prev_res_button:
        self.__queued_buttons.append("RES_BUTTON")


    # block button sends state
    if self.__state == 1:
      can_sends.append(nissancan.create_cruise_throttle_msg(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, self.frame, "NO_BUTTON_PRESSED"))
      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.06:
        if not self.__queued_buttons and self.ICBM.sendButton != SendButtonState.none:
          self.__queued_buttons.append(BUTTONS[self.ICBM.sendButton])
        # only check for icbm sends after giving the car time to adjust the set speed
        if self.__queued_buttons:
          self.__state = 2
        else:
          self.__state = 0

    # during button send state
    elif self.__state == 2:
      can_sends.append(nissancan.create_cruise_throttle_msg(packer, self.CP.carFingerprint, CS.cruise_throttle_msg, self.frame, self.__queued_buttons[0]))
      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.12:
        self.__queued_buttons.popleft()
        self.__state = 1
        self.last_button_frame = self.frame

    # idle state
    if self.__state == 0:
      if self.__queued_buttons or self.ICBM.sendButton != SendButtonState.none:
        self.__state = 1

    return can_sends
