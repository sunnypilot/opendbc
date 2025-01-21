from panda import Panda
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.nissan.values import CAR


class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.carName = "nissan"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.nissan)]
    ret.autoResumeSng = False

    ret.steerLimitTimer = 1.0

    ret.steerActuatorDelay = 0.1

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    if candidate == CAR.NISSAN_ALTIMA:
      # Altima has EPS on C-CAN unlike the others that have it on V-CAN
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_NISSAN_ALT_EPS_BUS

    # Used for panda safety and tests
    if candidate in (CAR.NISSAN_LEAF, CAR.NISSAN_LEAF_IC):
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_NISSAN_LEAF

    return ret

  @staticmethod
  def _get_params_sp(ret_stock: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], experimental_long: bool, docs: bool) -> tuple[structs.CarParams, structs.CarParamsSP]:
    return ret_stock, ret
