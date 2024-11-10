from enum import IntFlag


class HyundaiFlagsSP(IntFlag):
  """
  Sunnypilot flags for hyundai. We reuse CarParams from OP. So we define the flags in reverse order to avoid conflicts.
  """
  SP_ENHANCED_SCC = 2**31
  SP_CAN_LFA_BTN = 2**30
  SP_NAV_MSG = 2 ** 29

  SP_NON_SCC = 2 ** 28
  SP_NON_SCC_FCA = 2 ** 27
  SP_NON_SCC_RADAR_FCA = 2 ** 36

  SP_CAMERA_SCC_LEAD = 2 ** 25
  SP_LKAS12 = 2 ** 24
  SP_RADAR_TRACKS = 2 ** 23 # this one clashes with one in stock huindaiflags. We might need the new flags field afterall..
