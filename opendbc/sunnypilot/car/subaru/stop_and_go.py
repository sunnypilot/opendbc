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
    self.prev_cruise_state = 0
    self.epb_resume_frames_remaining = 0
    self.manual_hold = False

  def update_epb_resume_sequence(self, should_resume: bool) -> bool:
    if self.manual_parking_brake:
      return False

    if should_resume:
      self.epb_resume_frames_remaining = 5

    send_resume = self.epb_resume_frames_remaining > 0
    if self.epb_resume_frames_remaining > 0:
      self.epb_resume_frames_remaining -= 1

    return send_resume

  def update_stop_and_go(self, CC: structs.CarControl, CS: CarStateBase, frame: int) -> bool:
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

    if not CC.enabled or not CC.hudControl.leadVisible:
      return False

    close_distance = CS.es_distance_msg["Close_Distance"]
    in_standstill = CS.out.standstill
    pcm_standstill = CS.out.cruiseState.standstill

    if not in_standstill:
      self.last_standstill_frame = frame
      self.manual_hold = False

    # Check if we've been in standstill long enough
    standstill_duration = (frame - self.last_standstill_frame) * DT_CTRL
    in_standstill_hold = standstill_duration > 0.75

    # Car state distance-based conditions (EPB only)
    in_resume_distance = _SNG_ACC_MIN_DIST < close_distance < _SNG_ACC_MAX_DIST
    distance_increasing = close_distance > self.prev_close_distance
    distance_resume_allowed = in_resume_distance and distance_increasing

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      if self.manual_parking_brake:
        # Manual parking brake: Direct resume without sequence
        send_resume = in_standstill and in_standstill_hold

        if (frame - self.last_standstill_frame) * DT_CTRL >= 0.8:
          self.last_standstill_frame = frame
      else:
        # Pre-Global with EPB: Resume sequence with stock ACC distance conditions
        should_resume = in_standstill and distance_resume_allowed
        send_resume = self.update_epb_resume_sequence(should_resume)
    else:
      if self.manual_parking_brake:
        # Manual parking brake: Direct resume without sequence
        send_resume = in_standstill and in_standstill_hold
      else:
        # Global with EPB: Resume sequence with stock ACC distance conditions
        # Track manual hold state
        if in_standstill and pcm_standstill and self.prev_cruise_state == 1:
          self.manual_hold = True

        should_resume = pcm_standstill and not self.manual_hold and distance_resume_allowed
        send_resume = self.update_epb_resume_sequence(should_resume)

    self.prev_cruise_state = CS.cruise_state
    self.prev_close_distance = close_distance

    return send_resume

  def create_stop_and_go(self, packer, CC: structs.CarControl, CS: CarStateBase, pcm_cancel_cmd: bool, frame: int) -> list[CanData]:
    can_sends = []

    if not self.enabled:
      return can_sends

    send_resume = self.update_stop_and_go(CC, CS, frame)

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      can_sends.append(subarucan_ext.create_preglobal_throttle(packer, CS.throttle_msg, send_resume and not self.manual_parking_brake))

      if frame % 2 == 0:
        can_sends.append(subarucan_ext.create_preglobal_brake_pedal(packer, CS.brake_pedal_msg, send_resume and self.manual_parking_brake))
    else:
      can_sends.append(subarucan_ext.create_throttle(packer, CS.throttle_msg, send_resume and not self.manual_parking_brake))

      if frame % 2 == 0:
        can_sends.append(subarucan_ext.create_brake_pedal(packer, CS.brake_pedal_msg, pcm_cancel_cmd, send_resume and self.manual_parking_brake))

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
