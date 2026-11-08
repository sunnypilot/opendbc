from enum import IntEnum

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags

from opendbc.sunnypilot.car.hyundai.interceptors.ecu_interceptor import EcuInterceptorBase, \
  EcuInterceptorCarStateBase, EcuInterceptorCarControllerBase, EcuInterceptorRadarInterfaceBase
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

SAFETY_MODE_ADAS_SAFETY_HKG_ADAS_DRV_INTERCEPTOR = 5
ADAS_DRV_INTERCEPTOR_PROTOCOL_VERSION = 1
ADAS_INTERCEPTOR_HEARTBEAT_MSG = 0x1258


class ActvACISta(IntEnum):
  INIT = 0
  INACTIVE = 1
  ACTIVE = 2


class AdasDrvEcuInterceptor(EcuInterceptorBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.dcu_lfa_on = False

  @property
  def available(self):
    return self.CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR

  @property
  def trigger_msg(self):
    return 0x1A0

  @property
  def lfa_alt_steering(self):
    # the MDPS on angle steering cars only listens to LFA_ALT, so with the ADAS DRV intercepted we steer it directly
    return bool(self.available and self.CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING and self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG)

  @staticmethod
  def create_lfa_alt(packer, CAN, lat_active, apply_angle, apply_torque):
    values = {
      "ADAS_ActvACISta": ActvACISta.INIT,
      "ADAS_ActvACILvl2Sta": ActvACISta.ACTIVE if lat_active else ActvACISta.INACTIVE,
      "ADAS_StrAnglReqVal": apply_angle,
      "ADAS_ACIAnglTqRedcGainVal": apply_torque if lat_active else 0,
      "FCA_ESA_ActvSta": 0,
      "FCA_ESA_TqBstGainVal": 0,
    }
    return packer.make_can_msg("LFA_ALT", CAN.ECAN, values)

  def create_lkas_alt(self, packer, CAN, lkas_alt, dcu_lfa_icon, enabled, apply_angle):
    # the camera's copy, never engaged, so the ADAS DRV is never asked to steer. The rest passes through: the
    # ADAS DRV toggles LFA on the camera's LFA_BUTTON, not on CRUISE_BUTTONS. Once its LFA icon shows, the button
    # is dropped for the drive, so its LFA stays on for HDA and the side cars
    if not self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT:
      return []
    self.dcu_lfa_on = self.dcu_lfa_on or dcu_lfa_icon != 0
    values = {k: v for k, v in lkas_alt.items() if k not in ("CHECKSUM", "COUNTER")}
    values.update({
      "LKA_OptUsmSta": 0,
      "LKA_SysIndReq": 2 if enabled else 1,
      "StrTqReqVal": 0,
      "LKA_SysWrn": 0,
      "ActToiSta": 0,
      "LKA_UsmMod": 0,
      "LKA_RcgSta": 0,
      "Damping_Gain": 100,
      "ADAS_StrAnglReqVal": apply_angle,
      "LKAS_ANGLE_ACTIVE": 1,
      "ADAS_ACIAnglTqRedcGainVal": 0,
    })
    if self.dcu_lfa_on:
      values["LFA_BUTTON"] = 0
    return [packer.make_can_msg("LKAS_ALT", CAN.ACAN, values)]

  def create_adas_drv_intercept_msg(self, packer, CAN):
    values = {
      "REQUESTED_SAFETY_MODE": SAFETY_MODE_ADAS_SAFETY_HKG_ADAS_DRV_INTERCEPTOR,
      "REQUESTED_SAFETY_PARAM": ADAS_DRV_INTERCEPTOR_PROTOCOL_VERSION,
      "FEATURE_1": 1,
      "FEATURE_2": 1,
    }
    return [packer.make_can_msg("ADAS_DRV_INTERCEPT_OPT", CAN.ACAN, values)]


# VERY LIKELY NOT NEEDED AS WE DON'T KEEP A STATE ABOUT THE INTERCEPTOR, SINCE WE PASS MSGS AS-IS
class AdasDrvEcuInterceptorCarState(EcuInterceptorCarStateBase):
  pass


class AdasDrvEcuInterceptorCarController(EcuInterceptorCarControllerBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

  def initialize(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> AdasDrvEcuInterceptor:
    """
      This method is invoked to initialize the ESCC object.
      It should be called before any other methods are invoked.
    """
    return AdasDrvEcuInterceptor(CP, CP_SP)


class AdasDrvEcuInterceptorRadarInterface(EcuInterceptorRadarInterfaceBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

  def initialize(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> AdasDrvEcuInterceptor:
    """
      This method is invoked to initialize the ESCC object.
      It should be called before any other methods are invoked.
    """
    return AdasDrvEcuInterceptor(CP, CP_SP)
