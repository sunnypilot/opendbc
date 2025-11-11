"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP
from opendbc.car.common.conversions import Conversions as CV


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    speed_limit = cp_cam.vl["CAMERA_MESSAGES"]["SPEED_LIMIT_SIGN"]
    ret_sp.speedLimit = (speed_limit - 96.0) * 5.0 * CV.MPH_TO_MS if (speed_limit > 96 and speed_limit < 125) else 0.0

    if self.CP_SP.flags & HondaFlagsSP.NIDEC_HYBRID:
      ret.accFaulted = bool(cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_1"] or cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_2"])
      ret.stockAeb = bool(cp_cam.vl["BRAKE_COMMAND"]["AEB_REQ_1"] and cp_cam.vl["BRAKE_COMMAND"]["COMPUTER_BRAKE_HYBRID"] > 1e-5)

    if self.CP_SP.flags & HondaFlagsSP.HYBRID_ALT_BRAKEHOLD:
      ret.brakeHoldActive = cp.vl["BRAKE_HOLD_HYBRID_ALT"]["BRAKE_HOLD_ACTIVE"] == 1

    if self.CP_SP.enableGasInterceptor:
      # Same threshold as panda, equivalent to 1e-5 with previous DBC scaling
      gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = gas > 492
