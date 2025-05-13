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

  def update(self, CC: structs.CarControl, CS: CarStateBase, frame: int) -> bool:
    """
    Manages stop-and-go functionality for adaptive cruise control (ACC).

    Args:
        CC: Car control data
        CS: Car state data
        frame: Current frame number

    Returns:
        bool: Updated send_resume
    """

    if not self.enabled:
      return False

    send_resume = False

    hud_control = CC.hudControl

    close_distance = CS.es_distance_msg["Close_Distance"]
    lead_visible = hud_control.leadVisible
    is_standstill = CS.out.standstill
    is_pcm_standstill = CS.out.cruiseState.standstill

    if not is_standstill:
      self.last_standstill_frame = frame
      self.manual_hold = False

    is_in_standstill = (frame - self.last_standstill_frame) * DT_CTRL > 0.5  # In standstill for > 0.5 second
    in_resume_distance = _SNG_ACC_MIN_DIST < close_distance < _SNG_ACC_MAX_DIST
    distance_increasing = close_distance > self.prev_close_distance
    resume_allowed = in_resume_distance and distance_increasing

    # PREGLOBAL
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      # PREGLOBAL vehicles simply need to be stopped
      should_resume = is_standstill

    # non-PREGLOBAL
    else:
      if self.manual_parking_brake:
        # For manual parking brake, ensure vehicle has been stopped for a minimum time
        should_resume = is_standstill and is_in_standstill
      else:
        # For electronic parking brake, handle manual hold logic
        if (is_standstill and
           self.prev_cruise_state == 1 and
           is_pcm_standstill and
           not lead_visible):
          self.manual_hold = True

        # Only resume if ACC is in HOLD state (only with EPB) and not in manual hold state
        should_resume = is_pcm_standstill and not self.manual_hold

    # Apply resume command if criteria are met
    if CC.enabled and lead_visible and should_resume:
      if self.manual_parking_brake:
        # For manual parking brake, send brake message with non-zero speed in standstill to avoid non-EPB ACC disengage
        send_resume = True
      elif resume_allowed:
        # For vehicles with proper distance tracking, initiate ACC resume sequence
        self.sng_acc_resume = True

    if self.sng_acc_resume:
      if self.sng_acc_resume_cnt < 5:
        send_resume = True
        self.sng_acc_resume_cnt += 1
      else:
        self.sng_acc_resume = False
        self.sng_acc_resume_cnt = -1

    self.prev_cruise_state = CS.cruise_state
    self.prev_close_distance = close_distance

    return send_resume

  def create_stop_and_go(self, packer, CC, CS, pcm_cancel_cmd, frame):
    can_sends = []

    if not self.enabled:
      return can_sends

    send_resume = self.update(CC, CS, frame)

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      can_sends.append(subarucan_ext.create_preglobal_stop_and_go(packer, CS.throttle_msg, send_resume))
    else:
      if frame % 2 == 0 and self.manual_parking_brake:
        can_sends.append(subarucan_ext.create_stop_and_go_manual_parking_brake(packer, CS.brake_pedal_msg, pcm_cancel_cmd, send_resume))
      else:
        can_sends.append(subarucan_ext.create_stop_and_go(packer, CS.throttle, send_resume))

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
