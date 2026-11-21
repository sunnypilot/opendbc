"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.lateral import AngleSteeringLimitsVM
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LFA_ALT_MSG = 0xCB

# 175 deg: the MDPS faults above it. 2.2 deg per frame: below the MDPS 2.33 deg step check
LFA_ALT_ANGLE_LIMITS = AngleSteeringLimitsVM(175, MAX_ANGLE_RATE=2.2)

ACI_INACTIVE = 1
ACI_ACTIVE = 2
ACI_FLT_NONE = 0
ACI_FLT_TEMP_UNAVAILABLE = 4


def lfa_alt_steering(CP: structs.CarParams, CP_SP: structs.CarParamsSP, fingerprint, cam_bus: int) -> bool:
  return bool(CP_SP.flags & HyundaiFlagsSP.CANFD_ANGLE_STEERING and CP.flags & HyundaiFlags.CANFD_CAMERA_SCC and
              not CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and CP_SP.flags & HyundaiFlagsSP.CCNC and
              LFA_ALT_MSG in fingerprint[cam_bus])


def create_lfa_alt(packer, CAN, lat_active, apply_angle, tq_redc_gain):
  values = {
    "ADAS_ActvACISta": 0,
    "ADAS_ActvACILvl2Sta": ACI_ACTIVE if lat_active else ACI_INACTIVE,
    "ADAS_StrAnglReqVal": apply_angle,
    "ADAS_ACIAnglTqRedcGainVal": tq_redc_gain if lat_active else 0,
    "FCA_ESA_ActvSta": 0,
    "FCA_ESA_TqBstGainVal": 0,
  }
  return packer.make_can_msg("LFA_ALT", CAN.ECAN, values)


def mdps_aci_lv2_for_camera(mdps, lfa_alt) -> dict:
  # the camera faults when the MDPS level 2 state disagrees with its own LFA_ALT request, and turns its LFA on
  # unless the MDPS reports the interface unavailable. Real faults pass through
  values = {}
  requested = lfa_alt.get("ADAS_ActvACILvl2Sta")
  if requested in (ACI_INACTIVE, ACI_ACTIVE) and mdps.get("MDPS_ADASAciActvSta_Lv2") in (ACI_INACTIVE, ACI_ACTIVE):
    values["MDPS_ADASAciActvSta_Lv2"] = requested
  if mdps.get("MDPS_ADAS_AciFltSig_Lv2") == ACI_FLT_NONE:
    values["MDPS_ADAS_AciFltSig_Lv2"] = ACI_FLT_TEMP_UNAVAILABLE
  return values
