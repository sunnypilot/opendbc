from opendbc.car.structs import CarParams
from opendbc.car.hyundai.values import CAR

Ecu = CarParams.Ecu


FW_VERSIONS_EXT = {
  CAR.KIA_NIRO_EV: {
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00DE  MDPS C 1.00 1.05 56310Q4200\x00 4DEEC105',
    ],
  },
  CAR.HYUNDAI_KONA_EV_NON_SCC: {
    (Ecu.abs, 0x7d1, None): [
      b'\xf1\x00OS IEB \x02 212 \x11\x13 58520-K4000',
    ],
    (Ecu.eps, 0x7d4, None): [
      b'\xf1\x00OS  MDPS C 1.00 1.04 56310K4000\x00 4OEDC104',
    ],
    (Ecu.fwdCamera, 0x7c4, None): [
      b'\xf1\x00OSE LKAS AT USA LHD 1.00 1.00 95740-K4100 W40',
    ],
  },
}
