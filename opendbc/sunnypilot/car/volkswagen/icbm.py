"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import Enum, auto

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import CAR, VolkswagenFlags
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase
from opendbc.sunnypilot.car.volkswagen import mlbcan_ext, mqbcan_ext, pqcan_ext

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

SEND_REPETITIONS = 2
COOLDOWN_TIME = 0.2

# on these cars accel/decel jump the set speed to the next coarse step; resume/set move it by one unit
COARSE_STEP_CARS = (CAR.SEAT_ATECA_MK1, CAR.VOLKSWAGEN_GOLF_MK7)
COARSE_STEP = 10  # km/h; the imperial step is unproven, so imperial clusters press fine only


class State(Enum):
  IDLE = auto()
  SPAM = auto()
  COOLDOWN = auto()


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    if CP.flags & VolkswagenFlags.PQ:
      self.can_ext = pqcan_ext
    elif CP.flags & VolkswagenFlags.MLB:
      self.can_ext = mlbcan_ext
    else:
      self.can_ext = mqbcan_ext

    self.coarse_step_car = CP.carFingerprint in COARSE_STEP_CARS
    self.state = State.IDLE
    self.send_button = SendButtonState.none
    self.button = ""
    self.base_counter = 0
    self.repetition = 0

  def get_button(self, CS) -> str:
    increase = self.send_button == SendButtonState.increase
    if not self.coarse_step_car:
      return "increase" if increase else "decrease"

    if CS.is_metric:
      v_cruise = round(CS.out.cruiseState.speedCluster * CV.MS_TO_KPH)
      if increase and (v_cruise // COARSE_STEP + 1) * COARSE_STEP <= self.ICBM.vTarget:
        return "increase"
      if not increase and ((v_cruise - 1) // COARSE_STEP) * COARSE_STEP >= self.ICBM.vTarget:
        return "decrease"
    return "resume" if increase else "_set"

  def create_button_msg(self, packer, CAN, CS) -> CanData:
    gra_stock_values = CS.gra_stock_values.copy()
    gra_stock_values["COUNTER"] = self.base_counter + self.repetition
    self.last_button_frame = self.frame
    return self.can_ext.create_acc_buttons_control(packer, CAN, gra_stock_values, **{self.button: True})

  def update(self, CS, CC_SP, packer, CAN, frame, gra_send_ready) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame

    if self.state == State.IDLE:
      if self.ICBM.sendButton != SendButtonState.none and gra_send_ready:
        self.send_button = self.ICBM.sendButton
        self.button = self.get_button(CS)
        self.base_counter = CS.gra_stock_values["COUNTER"]
        self.repetition = 0
        self.state = State.SPAM
        can_sends.append(self.create_button_msg(packer, CAN, CS))

    elif self.state == State.SPAM:
      self.repetition += 1
      if gra_send_ready or self.repetition > SEND_REPETITIONS or self.ICBM.sendButton != self.send_button:
        self.state = State.COOLDOWN
      else:
        can_sends.append(self.create_button_msg(packer, CAN, CS))

    elif (self.frame - self.last_button_frame) * DT_CTRL > COOLDOWN_TIME:
      self.state = State.IDLE

    return can_sends
