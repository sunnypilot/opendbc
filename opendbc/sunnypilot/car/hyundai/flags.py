from enum import IntFlag


class HyundaiFlagsSP(IntFlag):
  """
  Sunnypilot flags for hyundai. We reuse CarParams from OP. So we define the flags in reverse order to avoid conflicts.
  """
  SP_ENHANCED_SCC = 2**32
  SP_CAN_LFA_BTN = 2**31
  SP_NAV_MSG = 2 ** 30

  SP_NON_SCC = 2 ** 29
  SP_NON_SCC_FCA = 2 ** 28
  SP_NON_SCC_RADAR_FCA = 2 ** 27

  SP_CAMERA_SCC_LEAD = 2 ** 26
  SP_LKAS12 = 2 ** 25
  SP_RADAR_TRACKS = 2 ** 24
