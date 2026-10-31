from enum import IntEnum

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags

from opendbc.sunnypilot.car.hyundai.interceptors.ecu_interceptor import EcuInterceptorBase, \
  EcuInterceptorCarStateBase, EcuInterceptorCarControllerBase, EcuInterceptorRadarInterfaceBase
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

SAFETY_MODE_ADAS_SAFETY_HKG_ADAS_DRV_INTERCEPTOR = 5
ADAS_DRV_INTERCEPTOR_PROTOCOL_VERSION = 1
ADAS_INTERCEPTOR_HEARTBEAT_MSG = 0x258


class ActvACISta(IntEnum):
  INIT = 0
  INACTIVE = 1
  ACTIVE = 2


class AdasDrvEcuInterceptor(EcuInterceptorBase):
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

  def create_adas_drv_intercept_msg(self, packer, CAN):
    values = {
      "REQUESTED_SAFETY_MODE": SAFETY_MODE_ADAS_SAFETY_HKG_ADAS_DRV_INTERCEPTOR,
      "REQUESTED_SAFETY_PARAM": ADAS_DRV_INTERCEPTOR_PROTOCOL_VERSION,
      "FEATURE_1": 1,
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
