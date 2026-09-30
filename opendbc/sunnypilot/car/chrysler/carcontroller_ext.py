from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.chrysler.values import CAR, RAM_DT
from opendbc.sunnypilot.car.chrysler.values_ext import ChryslerFlagsSP

GearShifter = structs.CarState.GearShifter

# Experimental settling interval, informed by jvePilot's 70-frame enable delay.
# This is not an EPS readiness measurement and requires vehicle validation.
WP_MAIN_ON_DELAY_NS = 700_000_000


class CarControllerExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.wp_main_delay = bool(CP_SP.flags & ChryslerFlagsSP.NO_MIN_STEERING_SPEED and CP.carFingerprint == CAR.JEEP_GRAND_CHEROKEE_2019)
    self.wp_main_on_ns: int | None = None
    self.wp_main_ready = False

  def update_wp_main_state(self, CS: CarStateBase, now_nanos: int) -> None:
    if not self.wp_main_delay:
      return

    # Advanced WP changes EPS speed spoofing with ACC MAIN availability. Observe
    # every control tick, including ticks without a steering CAN transmission.
    if not CS.out.cruiseState.available or not CS.out.canValid or CS.out.steerFaultTemporary or CS.out.steerFaultPermanent:
      self.wp_main_on_ns = None
      self.wp_main_ready = False
      return

    if self.wp_main_on_ns is None:
      self.wp_main_on_ns = now_nanos
    self.wp_main_ready = now_nanos - self.wp_main_on_ns >= WP_MAIN_ON_DELAY_NS

  def get_lkas_control_bit(self, CS: CarStateBase, CC: structs.CarControl, lkas_control_bit: bool) -> bool:
    if self.CP_SP.flags & ChryslerFlagsSP.NO_MIN_STEERING_SPEED:
      lkas_control_bit = CC.latActive and (not self.wp_main_delay or self.wp_main_ready)
    elif self.CP.carFingerprint in RAM_DT:
      if self.CP.minEnableSpeed <= CS.out.vEgo <= self.CP.minEnableSpeed + 0.5:
        lkas_control_bit = True
      if self.CP.minEnableSpeed >= 14.5 and CS.out.gearShifter != GearShifter.drive:
        lkas_control_bit = False

    return lkas_control_bit
