"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.nissan.values import CAR
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# Map generic send button states to CRUISE_THROTTLE button field names
BUTTON_FIELDS = {
  SendButtonState.increase: "RES_BUTTON",
  SendButtonState.decrease: "SET_BUTTON",
}

# Fields to copy from the latest CRUISE_THROTTLE message
CRUISE_THROTTLE_FIELDS = [
  "COUNTER",
  "PROPILOT_BUTTON",
  "CANCEL_BUTTON",
  "GAS_PEDAL_INVERTED",
  "SET_BUTTON",
  "RES_BUTTON",
  "FOLLOW_DISTANCE_BUTTON",
  "NO_BUTTON_PRESSED",
  "GAS_PEDAL",
  "USER_BRAKE_PRESSED",
  "USER_BRAKE_PRESSED_INVERTED",
  "NEW_SIGNAL_2",
  "GAS_PRESSED_INVERTED",
  "unsure1",
  "unsure2",
  "unsure3",
]


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def _create_button_msg(self, packer, CS, send_button_field: str) -> CanData:
    # Some Nissan platforms (e.g., Leaf/Leaf_IC) don't have all of these fields
    # present in their CRUISE_THROTTLE message. Use .get with a default to avoid KeyErrors
    values = {s: CS.cruise_throttle_msg.get(s, 0) for s in CRUISE_THROTTLE_FIELDS}

    # Set only the requested button, clear others
    values["CANCEL_BUTTON"] = 0
    values["NO_BUTTON_PRESSED"] = 0
    values["PROPILOT_BUTTON"] = 0
    values["SET_BUTTON"] = 1 if send_button_field == "SET_BUTTON" and not values["RES_BUTTON"] == 1 else 0
    values["RES_BUTTON"] = 1 if send_button_field == "RES_BUTTON" and not values["SET_BUTTON"] == 1 else 0
    values["FOLLOW_DISTANCE_BUTTON"] = 0

    can_bus = 1 if self.CP.carFingerprint == CAR.NISSAN_ALTIMA else 2
    return packer.make_can_msg("CRUISE_THROTTLE", can_bus, values)

  def update(self, CS, CC_SP, packer, frame, last_button_frame) -> list[CanData]:
    can_sends: list[CanData] = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton != SendButtonState.none:
      send_field = BUTTON_FIELDS[self.ICBM.sendButton]

      #send every 0.2 sec
      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.2:
        can_sends.append(self._create_button_msg(packer, CS, send_field))
        self.last_button_frame = self.frame

    return can_sends
