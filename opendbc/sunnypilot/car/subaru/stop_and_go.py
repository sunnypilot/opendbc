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
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP
from opendbc.can.parser import CANParser


class SnGCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.enabled = CP_SP.flags & (SubaruFlagsSP.STOP_AND_GO | SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE)
    self.manual_parking_brake = CP_SP.flags & SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE

    self.last_standstill_frame = 0
    self.epb_resume_frames_remaining = 0

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

    in_standstill = CS.out.standstill

    if not in_standstill:
      self.last_standstill_frame = frame

    # Check if we've been in standstill long enough
    standstill_duration = (frame - self.last_standstill_frame) * DT_CTRL
    in_standstill_hold = standstill_duration > 0.75
    if (frame - self.last_standstill_frame) * DT_CTRL >= 0.8:
      self.last_standstill_frame = frame

    if self.manual_parking_brake:
      # Manual parking brake: Direct resume when the standstill hold threshold is reached to prevent ACC fault
      send_resume = in_standstill_hold
    else:
      # EPB: Resume sequence with planner resume desire
      send_resume = self.update_epb_resume_sequence(CC.cruiseControl.resume)

    return send_resume

  def create_stop_and_go(self, packer, CC: structs.CarControl, CS: CarStateBase, frame: int) -> list[CanData]:
    can_sends = []

    if not self.enabled:
      return can_sends

    send_resume = self.update_stop_and_go(CC, CS, frame)

    if send_resume:
      if self.manual_parking_brake:
        can_sends.append(subarucan_ext.create_brake_pedal(packer, self.CP, CS.brake_pedal_msg))
      else:
        can_sends.append(subarucan_ext.create_throttle(packer, self.CP, CS.throttle_msg))

    return can_sends


class SnGCarState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.enabled = CP_SP.flags & (SubaruFlagsSP.STOP_AND_GO | SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE)
    self.brake_pedal_msg: dict[str, float] = {}
    self.throttle_msg: dict[str, float] = {}

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    if not self.enabled:
      return

    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret.cruiseState.standstill = ret.standstill

    self.brake_pedal_msg = copy.copy(cp.vl["Brake_Pedal"])
    self.throttle_msg = copy.copy(cp.vl["Throttle"])
