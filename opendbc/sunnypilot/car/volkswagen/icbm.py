"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import CAR, CarControllerParams, VolkswagenFlags
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase
from opendbc.sunnypilot.car.volkswagen import mlbcan_ext, mqbcan_ext, pqcan_ext

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BURST_FRAMES = 3
COOLDOWN_TIME = 0.2

ACCEL_DECEL = {
  SendButtonState.increase: ButtonType.accelCruise,
  SendButtonState.decrease: ButtonType.decelCruise,
}
RESUME_SET = {
  SendButtonState.increase: ButtonType.resumeCruise,
  SendButtonState.decrease: ButtonType.setCruise,
}

# on these cars accel/decel jump the set speed to the next coarse step; resume/set move it by one unit
COARSE_STEP_CARS = (CAR.SEAT_ATECA_MK1, CAR.VOLKSWAGEN_GOLF_MK7)
COARSE_STEP = 10  # km/h; the imperial step is unproven, so imperial clusters press fine only


def coarse_step_fits(v_cruise: int, v_target: float, increase: bool) -> bool:
  if increase:
    return (math.floor(v_cruise / COARSE_STEP) + 1) * COARSE_STEP <= v_target
  return (math.ceil(v_cruise / COARSE_STEP) - 1) * COARSE_STEP >= v_target


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    if CP.flags & VolkswagenFlags.PQ:
      self.can_ext = pqcan_ext
    elif CP.flags & VolkswagenFlags.MLB:
      self.can_ext = mlbcan_ext
    else:
      self.can_ext = mqbcan_ext

    self.button_signals = {b.event_type: b.can_msg for b in CarControllerParams(CP).BUTTONS}
    self.coarse_step_car = CP.carFingerprint in COARSE_STEP_CARS
    self.request = SendButtonState.none
    self.button = ButtonType.unknown
    self.counter = 0
    self.frames_left = 0

  def choose_button(self, CS) -> ButtonType:
    if not self.coarse_step_car:
      return ACCEL_DECEL[self.request]

    if CS.is_metric:
      v_cruise = round(CS.out.cruiseState.speedCluster * CV.MS_TO_KPH)
      if coarse_step_fits(v_cruise, self.ICBM.vTarget, self.request == SendButtonState.increase):
        return ACCEL_DECEL[self.request]
    return RESUME_SET[self.request]

  def update(self, CS, CC_SP, packer, CAN, frame, gra_send_ready) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame

    if gra_send_ready or self.ICBM.sendButton != self.request:
      self.frames_left = 0

    cooldown_done = (self.frame - self.last_button_frame) * DT_CTRL > COOLDOWN_TIME
    if self.frames_left == 0 and gra_send_ready and self.ICBM.sendButton != SendButtonState.none and cooldown_done:
      self.request = self.ICBM.sendButton
      self.button = self.choose_button(CS)
      self.counter = CS.gra_stock_values["COUNTER"]
      self.frames_left = BURST_FRAMES

    if self.frames_left > 0:
      gra_stock_values = CS.gra_stock_values.copy()
      gra_stock_values["COUNTER"] = self.counter
      can_sends.append(self.can_ext.create_acc_button_press(packer, CAN, gra_stock_values, self.button_signals[self.button]))
      self.counter += 1
      self.frames_left -= 1
      self.last_button_frame = self.frame

    return can_sends
