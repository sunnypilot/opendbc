"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import copy
from enum import StrEnum

from opendbc.car import Bus, structs, DT_CTRL
from opendbc.car.can_definitions import CanData
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
    self.last_resume_frame = 0
    self.manual_hold = False
    self.prev_cruise_state = 0

  def update(self, CC: structs.CarControl, CS: CarStateBase, frame: int) -> bool:
    """
    Manages stop-and-go functionality for adaptive cruise control (ACC).

    Args:
        CC: Car control data
        CS: Car state data
        frame: Current frame number

    Returns:
        bool: True if resume command should be sent, False otherwise
    """

    if not self.enabled:
      return False

    close_distance = CS.es_distance_msg["Close_Distance"]
    lead_visible = CC.hudControl.leadVisible
    is_standstill = CS.out.standstill
    is_pcm_standstill = CS.out.cruiseState.standstill

    if not is_standstill:
      self.last_standstill_frame = frame
      self.manual_hold = False

    is_in_standstill = (frame - self.last_standstill_frame) * DT_CTRL > 0.5  # In standstill for > 0.5 second
    in_resume_distance = _SNG_ACC_MIN_DIST < close_distance < _SNG_ACC_MAX_DIST
    distance_increasing = close_distance > self.prev_close_distance
    resume_allowed = in_resume_distance and distance_increasing

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      should_resume = is_standstill
    else:
      if self.manual_parking_brake:
        should_resume = is_standstill and is_in_standstill
      else:
        # For EPB vehicles, track manual hold state
        if (is_standstill and
           self.prev_cruise_state == 1 and
           is_pcm_standstill and
           not lead_visible):
          self.manual_hold = True

        should_resume = is_pcm_standstill and not self.manual_hold

    send_resume = False
    if CC.enabled and lead_visible and should_resume:
      if self.manual_parking_brake:
        send_resume = True
      else:
        self.sng_acc_resume = resume_allowed

    # Handle ACC resume sequence state machine
    if not self.sng_acc_resume:
      self.last_resume_frame = frame
    else:
      send_resume = (frame - self.last_resume_frame) < 5
      if not send_resume:
        self.sng_acc_resume = False

    self.prev_cruise_state = CS.cruise_state
    self.prev_close_distance = close_distance

    return send_resume

  def create_stop_and_go(self, packer, CC: structs.CarControl, CS: CarStateBase, pcm_cancel_cmd: bool, frame: int) -> list[CanData]:
    can_sends = []

    if not self.enabled:
      return can_sends

    send_resume = self.update(CC, CS, frame)

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      can_sends.append(subarucan_ext.create_preglobal_stop_and_go(packer, CS.throttle_msg, send_resume))
    else:
      if self.manual_parking_brake and frame % 2 == 0:
        can_sends.append(subarucan_ext.create_stop_and_go_manual_parking_brake(packer, CS.brake_pedal_msg,
                                                                               pcm_cancel_cmd, send_resume))
      else:
        can_sends.append(subarucan_ext.create_stop_and_go(packer, CS.throttle_msg, send_resume))

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
