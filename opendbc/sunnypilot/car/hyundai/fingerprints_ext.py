from opendbc.car.structs import CarParams
from opendbc.car.hyundai.values import CAR

Ecu = CarParams.Ecu


FW_VERSIONS_EXT = {
  CAR.KIA_NIRO_EV: {
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00DE  MDPS C 1.00 1.05 56310Q4200\x00 4DEEC105',
    ],
  },
  CAR.GENESIS_G90_2ND_GEN: {
    (Ecu.fwdRadar, 0x7d0, None): [
      b'\xf1\x00RS4_ RDR -----      1.00 1.00 99110-T4300         ',
    ],
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00RS4 MFC  AT USA LHD 1.00 1.03 99211-T4000 220315',
    ],
  },
}
