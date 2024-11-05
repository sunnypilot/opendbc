from enum import IntFlag


class HyundaiFlagsSP(IntFlag):
  SP_ENHANCED_SCC = 1
  SP_CAN_LFA_BTN = 2
  SP_NAV_MSG = 2 ** 2

  SP_NON_SCC = 2 ** 3
  SP_NON_SCC_FCA = 2 ** 4
  SP_NON_SCC_RADAR_FCA = 2 ** 5

  SP_CAMERA_SCC_LEAD = 2 ** 6
  SP_LKAS12 = 2 ** 7
  SP_RADAR_TRACKS = 2 ** 8
  SP_UPSTREAM_TACO = 2 ** 9
