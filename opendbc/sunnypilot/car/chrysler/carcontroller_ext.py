"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import CarStateBase
from opendbc.car.chrysler import chryslercan
from opendbc.car.chrysler.values import RAM_DT, RAM_CARS
from opendbc.sunnypilot.car.chrysler.values import ChryslerFlagsSP

GearShifter = structs.CarState.GearShifter


class CarControllerExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self._last_button_frame = 0

  def get_lkas_control_bit(self, CS: CarStateBase, CC: structs.CarControl, lkas_control_bit: bool, lkas_control_bit_prev: bool) -> bool:
    if self.CP_SP.flags & ChryslerFlagsSP.NO_MIN_STEERING_SPEED:
      lkas_control_bit = CC.latActive
    elif self.CP.carFingerprint in RAM_DT:
      if not lkas_control_bit:
        lkas_control_bit = lkas_control_bit_prev
        if self.CP.minEnableSpeed <= CS.out.vEgo <= self.CP.minEnableSpeed + 0.5:
          lkas_control_bit = True
        if self.CP.minEnableSpeed >= 14.5 and CS.out.gearShifter != GearShifter.drive:
          lkas_control_bit = False

    return lkas_control_bit

  def ram_resume(self, CC: structs.CarControl, CS: CarStateBase, packer, last_button_frame) -> list[CanData]:
    can_sends = []

    if self.CP.carFingerprint not in RAM_CARS:
      return can_sends

    das_bus = 2

    if CS.button_counter != self._last_button_frame:
      self._last_button_frame = last_button_frame

      if CC.cruiseControl.resume:
        can_sends.append(chryslercan.create_cruise_buttons(packer, CS.button_counter, das_bus, self.CP, resume=True))

    return can_sends
