from cereal import car
from opendbc.car.structs import CarParams
from opendbc.car.volvo.values import CAR

Ecu = car.CarParams.Ecu

FW_VERSIONS = {
  CAR.VOLVO_V60: {
    (Ecu.engine, 0x7e0, None): [
      b'PLACEHOLDER',
    ],
  }
}