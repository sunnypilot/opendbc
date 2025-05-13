"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import copy
from enum import StrEnum

from opendbc.car import Bus, structs, DT_CTRL
from opendbc.car.interfaces import CarStateBase

from opendbc.car.subaru.values import SubaruFlags
from opendbc.sunnypilot.car.subaru import subarucan_ext
from opendbc.sunnypilot.car.subaru.values import SubaruFlagsSP
from opendbc.can.parser import CANParser

_SNG_ACC_MIN_DIST = 3
_SNG_ACC_MAX_DIST = 4.5


class SnGCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.enabled = CP_SP.flags & (SubaruFlagsSP.STOP_AND_GO | SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE)
    self.manual_parking_brake = CP_SP.flags & SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE
    self.prev_close_distance = 0
    self.last_standstill_frame = 0
    self.sng_acc_resume = False
    self.sng_acc_resume_cnt = -1
    self.manual_hold = False
    self.prev_cruise_state = 0

    self.throttle_cmd = False
    self.speed_cmd = False

  def update(self, CC: structs.CarControl, CS: CarStateBase, frame: int,
             throttle_cmd: bool = False, speed_cmd: bool = False) -> None:
    """
    Manages stop-and-go functionality for adaptive cruise control (ACC).

    Args:
        CC: Car control data
        CS: Car state data
        frame: Current frame number
        throttle_cmd: Initial throttle command state
        speed_cmd: Initial speed command state

    Returns:
        tuple[bool, bool]: Updated (throttle_cmd, speed_cmd)
    """

    if not self.enabled:
      return

    hud_control = CC.hudControl

    close_distance = CS.es_distance_msg["Close_Distance"]
    lead_visible = hud_control.leadVisible
    cruise_state = CS.cruise_state
    is_standstill = CS.out.standstill
    in_resume_distance = _SNG_ACC_MIN_DIST < close_distance < _SNG_ACC_MAX_DIST  # Above minimum and below maximum resume distance
    distance_increasing = close_distance > self.prev_close_distance  # Distance with lead car is increasing
    resume_allowed = in_resume_distance and distance_increasing

    if not is_standstill:
      self.last_standstill_frame = frame
      self.manual_hold = False
    is_in_standstill = (frame - self.last_standstill_frame) * DT_CTRL > 0.5  # Standstill for > 0.5 second

    should_resume = False
    # PREGLOBAL
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      # Initiate the ACC resume sequence if conditions are met
      if is_standstill:
        should_resume = True

    # non-PREGLOBAL
    else:
      if self.manual_parking_brake:
        # Send brake message with non-zero speed in standstill to avoid non-EPB ACC disengage
        if (CC.enabled and                   # ACC active
           lead_visible and                      # Lead car present
           is_standstill and                     # Vehicle is stopped
           is_in_standstill):
          speed_cmd = True
      else:
        # Electric parking brake
        # Record manual hold when stopped with no car in front and cruise state changes appropriately
        if (is_standstill and
           self.prev_cruise_state == 1 and
           CS.out.cruiseState.standstill and
           not lead_visible):
          self.manual_hold = True

        # Initiate the ACC resume sequence if conditions are met
        if CS.out.cruiseState.standstill and not self.manual_hold:  # ACC HOLD (only with EPB) and not in manual hold
          should_resume = True

    if CC.enabled and lead_visible and resume_allowed and should_resume:
      self.sng_acc_resume = True

    if self.sng_acc_resume:
      if self.sng_acc_resume_cnt < 5:
        throttle_cmd = True
        self.sng_acc_resume_cnt += 1
      else:
        self.sng_acc_resume = False
        self.sng_acc_resume_cnt = -1

    self.prev_cruise_state = cruise_state
    self.prev_close_distance = close_distance

    self.throttle_cmd = throttle_cmd
    self.speed_cmd = speed_cmd

  def create_stop_and_go(self, packer, CS, pcm_cancel_cmd, frame):
    can_sends = []

    if not self.enabled:
      return can_sends

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      can_sends.append(subarucan_ext.create_preglobal_stop_and_go(packer, CS.throttle_msg, self.throttle_cmd))
    else:
      if frame % 2 == 0 and self.manual_parking_brake:
        can_sends.append(subarucan_ext.create_stop_and_go_manual_parking_brake(packer, CS.brake_pedal_msg, pcm_cancel_cmd, self.speed_cmd))
      else:
        can_sends.append(subarucan_ext.create_stop_and_go(packer, CS.throttle, self.throttle_cmd))

    return can_sends


class SnGCarState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.enabled = CP_SP.flags & (SubaruFlagsSP.STOP_AND_GO | SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE)
    self.cruise_state: float = 0
    self.brake_pedal_msg: dict[str, float] = {}
    self.throttle_msg: dict[str, float] = {}

  def update(self, can_parsers: dict[StrEnum, CANParser]) -> None:
    if not self.enabled:
      return

    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if not self.CP.flags & SubaruFlags.PREGLOBAL:
      self.cruise_state = cp_cam.vl["ES_DashStatus"]["Cruise_State"]
      self.brake_pedal_msg = copy.copy(cp.vl["Brake_Pedal"])

    self.throttle_msg = copy.copy(cp.vl["Throttle"])
