"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections import namedtuple
from enum import IntFlag

from opendbc.car import structs

ButtonType = structs.CarState.ButtonEvent.Type
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])

BUTTONS = [
  Button(ButtonType.accelCruise, "WheelButtons", "RightButton_RightClick", [2]),
  Button(ButtonType.decelCruise, "WheelButtons", "RightButton_LeftClick", [2]),
]


class RivianFlagsSP(IntFlag):
  LONGITUDINAL_HARNESS_UPGRADE = 1
