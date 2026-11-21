from opendbc.car.structs import CarParams
from opendbc.car.ford.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS_EXT = {
  CAR.FORD_F_150_MK14_5: {
    (Ecu.eps, 0x730, None): [
      b'RL14-14D003-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'SJ8T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'SJ8T-14H102-ABR\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
}
