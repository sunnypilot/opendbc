"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import copy
from enum import StrEnum

from opendbc.car import Bus, structs
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
    self.prev_standstill = False
    self.standstill_start = 0
    self.sng_acc_resume = False
    self.sng_acc_resume_cnt = -1
    self.manual_hold = False
    self.prev_cruise_state = 0

    self.throtle_cmd = False
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

    close_distance = CS.es_distance_msg["Close_Distance"]
    has_lead_car = CS.es_distance_msg["Car_Follow"] == 1
    cruise_state = CS.cruise_state
    is_standstill = CS.out.standstill
    is_acc_enabled = CC.enabled
    distance_increasing = close_distance > self.prev_close_distance

    # PREGLOBAL
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      # Initiate the ACC resume sequence if conditions are met
      if (is_acc_enabled and                                         # ACC active
         has_lead_car and                                            # Lead car present
         is_standstill and                                           # Must be standing still
         _SNG_ACC_MIN_DIST < close_distance < _SNG_ACC_MAX_DIST and  # Above minimum and below maximum resume distance
         distance_increasing):                                       # Distance with lead car is increasing
        self.sng_acc_resume = True

    # non-PREGLOBAL
    else:
      if self.manual_parking_brake:
        # Send brake message with non-zero speed in standstill to avoid non-EPB ACC disengage
        if (is_acc_enabled and                   # ACC active
           has_lead_car and                      # Lead car present
           is_standstill and                     # Vehicle is stopped
           frame > self.standstill_start + 50):  # Standstill for >0.5 second
          speed_cmd = True
      else:
        # Electric parking brake
        # Record manual hold when stopped with no car in front and cruise state changes appropriately
        if (is_standstill and
           self.prev_cruise_state == 1 and
           cruise_state == 3 and
           not has_lead_car):
          self.manual_hold = True

        # Cancel manual hold when car starts moving
        if not is_standstill:
          self.manual_hold = False

        # Initiate the ACC resume sequence if conditions are met
        if (is_acc_enabled and                                         # ACC active
           not self.manual_hold and                                    # Not in manual hold
           has_lead_car and                                            # Lead car present
           cruise_state == 3 and                                       # ACC HOLD (only with EPB)
           _SNG_ACC_MIN_DIST < close_distance < _SNG_ACC_MAX_DIST and  # Above minimum and below maximum resume distance
           distance_increasing):                                       # Distance with lead car is increasing
          self.sng_acc_resume = True

      if is_standstill and not self.prev_standstill:
        self.standstill_start = frame
      self.prev_standstill = is_standstill
      self.prev_cruise_state = cruise_state

    if self.sng_acc_resume:
      if self.sng_acc_resume_cnt < 5:
        throttle_cmd = True
        self.sng_acc_resume_cnt += 1
      else:
        self.sng_acc_resume = False
        self.sng_acc_resume_cnt = -1

    self.prev_close_distance = close_distance

    self.throtle_cmd = throttle_cmd
    self.speed_cmd = speed_cmd

  def create_stop_and_go(self, packer, CS, pcm_cancel_cmd, frame):
    can_sends = []

    if not self.enabled:
      return can_sends

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      can_sends.append(subarucan_ext.create_preglobal_throttle(packer, CS.throttle_msg["COUNTER"] + 1, CS.throttle_msg, self.throtle_cmd))
    else:
      can_sends.append(subarucan_ext.create_throttle(packer, CS.throttle_msg["COUNTER"] + 1, CS.throttle_msg, self.throtle_cmd))

      if frame % 2 == 0:
        can_sends.append(subarucan_ext.create_brake_pedal(packer, frame // 2, CS.brake_pedal_msg, self.speed_cmd, pcm_cancel_cmd))

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
