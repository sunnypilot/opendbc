"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.gm.values_ext import GMFlagsSP


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.single_pedal_mode = False
    self.pedal_steady = 0.

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    pt_cp = can_parsers[Bus.pt]

    if self.CP_SP.flags & GMFlagsSP.NON_ACC:
      ret.cruiseState.enabled = pt_cp.vl["ECMCruiseControl"]["CruiseActive"] != 0
      ret.cruiseState.speed = pt_cp.vl["ECMCruiseControl"]["CruiseSetSpeed"] * CV.KPH_TO_MS
      ret.accFaulted = False

  def update_gas_pressed(self, ret: structs.CarState, pt_cp) -> structs.CarState:
    """Update gas pressed for pedal interceptor vehicles"""
    from opendbc.car.gm.values import CAMERA_ACC_CAR
    if self.CP.enableGasInterceptorDEPRECATED:
      gas = (pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
      # Panda 515 threshold = 10.88. Set lower to avoid panda blocking messages and GasInterceptor faulting.
      threshold = 20 if self.CP.carFingerprint in CAMERA_ACC_CAR else 4
      ret.gasPressed = gas > threshold
    else:
      ret.gasPressed = pt_cp.vl["AcceleratorPedal2"]["AcceleratorPedal2"] / 254. > 1e-5
    return ret

  def update_regen_braking(self, ret: structs.CarState, pt_cp) -> structs.CarState:
    """Update regen braking for pedal interceptor vehicles"""
    if self.CP.transmissionType == structs.CarParams.TransmissionType.direct:
      ret.regenBraking = pt_cp.vl["EBCMRegenPaddle"]["RegenPaddle"] != 0
      self.single_pedal_mode = ret.gearShifter == structs.CarState.GearShifter.low or pt_cp.vl["EVDriveMode"]["SinglePedalModeActive"] == 1
    return ret
