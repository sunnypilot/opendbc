from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS_EXT = {
  CAR.TESLA_MODEL_X: {
    (Ecu.eps, 0x730, None): [
      b'TeM3_SP_XP002p2_0.0.0 (36),XPR003.10.0',
    ],
  },
}
