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

  def stop_and_go(self, CC: structs.CarControl, CS: CarStateBase, frame: int,
                  throttle_cmd: bool = False, speed_cmd: bool = False) -> tuple[bool, bool]:
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
      return throttle_cmd, speed_cmd

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      # Initiate the ACC resume sequence if conditions are met
      if (CC.enabled                                          # ACC active
        and CS.es_distance_msg["Car_Follow"] == 1             # lead car
        and CS.out.standstill                                 # must be standing still
        and CS.es_distance_msg["Close_Distance"] > _SNG_ACC_MIN_DIST             # acc resume trigger low threshold
        and CS.es_distance_msg["Close_Distance"] < _SNG_ACC_MAX_DIST             # acc resume trigger high threshold
        and CS.es_distance_msg["Close_Distance"] > self.prev_close_distance):    # distance with lead car is increasing
        self.sng_acc_resume = True
    else:
      if self.manual_parking_brake:
        # Send brake message with non-zero speed in standstill to avoid non-EPB ACC disengage
        if (CC.enabled                                        # ACC active
          and CS.es_distance_msg["Car_Follow"] == 1           # lead car
          and CS.out.standstill
          and frame > self.standstill_start + 50):       # standstill for >0.5 second
          speed_cmd = True
      else:
        # Record manual hold set while in standstill and no car in front
        if CS.out.standstill and self.prev_cruise_state == 1 and CS.cruise_state == 3 and CS.es_distance_msg["Car_Follow"] == 0:
          self.manual_hold = True
        # Cancel manual hold when car starts moving
        if not CS.out.standstill:
          self.manual_hold = False
        # Initiate the ACC resume sequence if conditions are met
        if (CC.enabled                                        # ACC active
          and not self.manual_hold
          and CS.es_distance_msg["Car_Follow"] == 1           # lead car
          and CS.cruise_state == 3                            # ACC HOLD (only with EPB)
          and CS.es_distance_msg["Close_Distance"] > _SNG_ACC_MIN_DIST           # acc resume trigger low threshold
          and CS.es_distance_msg["Close_Distance"] < _SNG_ACC_MAX_DIST           # acc resume trigger high threshold
          and CS.es_distance_msg["Close_Distance"] > self.prev_close_distance):  # distance with lead car is increasing
          self.sng_acc_resume = True

      if CS.out.standstill and not self.prev_standstill:
        self.standstill_start = frame
      self.prev_standstill = CS.out.standstill
      self.prev_cruise_state = CS.cruise_state

    if self.sng_acc_resume:
      if self.sng_acc_resume_cnt < 5:
        throttle_cmd = True
        self.sng_acc_resume_cnt += 1
      else:
        self.sng_acc_resume = False
        self.sng_acc_resume_cnt = -1

    self.prev_close_distance = CS.es_distance_msg["Close_Distance"]

    return throttle_cmd, speed_cmd


class SnGCarState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.enabled = CP_SP.flags & (SubaruFlagsSP.STOP_AND_GO | SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE)
    self.cruise_state = 0
    self.brake_pedal_msg = None
    self.throttle_msg = None

  def update(self, can_parsers: dict[StrEnum, CANParser]) -> None:
    if not self.enabled:
      return

    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if not self.CP.flags & SubaruFlags.PREGLOBAL and self.CP_SP.flags & SubaruFlagsSP.STOP_AND_GO:
      self.cruise_state = cp_cam.vl["ES_DashStatus"]["Cruise_State"]
      self.brake_pedal_msg = copy.copy(cp.vl["Brake_Pedal"])
      self.throttle_msg = copy.copy(cp.vl["Throttle"])
