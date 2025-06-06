"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.rivian.values import DBC

ButtonType = structs.CarState.ButtonEvent.Type

MAX_SET_SPEED = 85 * CV.MPH_TO_MS
MIN_SET_SPEED = 20 * CV.MPH_TO_MS



class CarStateExt:
  def __init__(self, CP: structs.CarParams):
    self.CP = CP

    self.set_speed = 10
    self.increase_btn_pressed_prev = False
    self.increase_cntr = 0
    self.decrease_btn_pressed_prev = False
    self.decrease_cntr = 0
    self.distance_button = 0

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    if self.CP.openpilotLongitudinalControl:
      cp_park = can_parsers[Bus.alt]

      # distance scroll wheel
      current_right_scroll = cp_park.vl["WheelButtons"]["RightButton_Scroll"]
      if current_right_scroll != 255:
        if self.distance_button != current_right_scroll:
          ret.buttonEvents = [structs.CarState.ButtonEvent(pressed=False, type=ButtonType.gapAdjustCruise)]
        self.distance_button = current_right_scroll

      # button logic for set-speed
      increase_btn_pressed_now = cp_park.vl["WheelButtons"]["RightButton_RightClick"] == 2
      decrease_btn_pressed_now = cp_park.vl["WheelButtons"]["RightButton_LeftClick"] == 2

      self.increase_cntr = self.increase_cntr + 1 if increase_btn_pressed_now else 0
      self.decrease_cntr = self.decrease_cntr + 1 if decrease_btn_pressed_now else 0

      set_speed_mph = self.set_speed * CV.MS_TO_MPH
      if increase_btn_pressed_now and self.increase_cntr % 66 == 0:
        self.set_speed = (int(math.ceil((set_speed_mph + 1) / 5.0)) * 5) * CV.MPH_TO_MS
      elif not self.increase_btn_pressed_prev and increase_btn_pressed_now:
        self.set_speed += CV.MPH_TO_MS

      if decrease_btn_pressed_now and self.decrease_cntr % 66 == 0:
        self.set_speed = (int(math.floor((set_speed_mph - 1) / 5.0)) * 5) * CV.MPH_TO_MS
      elif not self.decrease_btn_pressed_prev and decrease_btn_pressed_now:
        self.set_speed -= CV.MPH_TO_MS

      self.increase_btn_pressed_prev = increase_btn_pressed_now
      self.decrease_btn_pressed_prev = decrease_btn_pressed_now

      if not ret.cruiseState.enabled:
        self.set_speed = ret.vEgo

      self.set_speed = max(MIN_SET_SPEED, min(self.set_speed, MAX_SET_SPEED))
      ret.cruiseState.speed = self.set_speed

      ret.leftBlindspot = cp_park.vl["BSM_BlindSpotIndicator"]["BSM_BlindSpotIndicator_Left"] != 0
      ret.rightBlindspot = cp_park.vl["BSM_BlindSpotIndicator"]["BSM_BlindSpotIndicator_Right"] != 0

  @staticmethod
  def get_parser(CP, messages) -> None:
    alt_messages = []

    # only available with the longitudinal upgrade
    if CP.openpilotLongitudinalControl:
      alt_messages += [
        ("WheelButtons", 20),
        ("BSM_BlindSpotIndicator", 20),
      ]

    messages[Bus.alt] = CANParser(DBC[CP.carFingerprint][Bus.alt], alt_messages, 5)
