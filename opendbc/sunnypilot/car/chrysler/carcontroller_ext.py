from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.chrysler.values import RAM_DT
from opendbc.sunnypilot.car.chrysler.values import ChryslerFlagsSP

GearShifter = structs.CarState.GearShifter


class CarControllerExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

  def get_lkas_control_bit(self, CS: CarStateBase, CC: structs.CarControl, lkas_control_bit: bool, lkas_control_bit_prev: bool) -> bool:
    if self.CP_SP.flags & ChryslerFlagsSP.NO_MIN_STEERING_SPEED:
      lkas_control_bit = CC.latActive

    return lkas_control_bit
