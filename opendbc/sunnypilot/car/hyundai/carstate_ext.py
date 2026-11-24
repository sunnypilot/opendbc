"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import copy
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, CAR, DBC
from opendbc.sunnypilot.car.hyundai.ccnc import ccnc_hud_enabled
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

# CCNC_0x161 ALERTS_1
DCU_EMERGENCY_BRAKING_ALERTS = range(11, 16)
DCU_EMERGENCY_STEERING_ALERTS = range(21, 27)


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.aBasis = 0.0
    self.msg_161 = {}
    self.msg_162 = {}
    self.msg_1e0 = {}
    self.mdps = {}
    self.cruise_buttons_msg = {}
    self.lkas_alt = {}
    self.lfa_alt = {}

  def update_speed_limit(self, cp, cp_cam) -> float:
    speed_limit = 0

    if self.CP.flags & HyundaiFlags.CANFD:
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        bus = cp if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else cp_cam
        speed_limit = bus.vl["FR_CMR_02_100ms"]["ISLW_SpdCluMainDis"]
    else:
      nav, cam = 0, 0
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        nav = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]
      if self.CP_SP.flags & HyundaiFlagsSP.HAS_LKAS12:
        cam = cp_cam.vl["LKAS12"]["CF_Lkas_TsrSpeed_Display_Clu"]

      speed_limit = cam if cam not in (0, 255) else nav

    if speed_limit in (0, 255):
      speed_limit = 0

    return speed_limit

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser], speed_conv: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS13"]["aBasis"]

    if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC:
      cruise_msg = "LABEL11" if self.CP.flags & HyundaiFlags.EV else \
                   "E_CRUISE_CONTROL" if self.CP.flags & HyundaiFlags.HYBRID else \
                   "EMS16"
      cruise_available_sig = "CC_React" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_M"
      cruise_enabled_sig = "CC_ACT" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_S"
      cruise_speed_msg = "E_EMS11" if self.CP.flags & HyundaiFlags.EV else \
                         "ELECT_GEAR" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "EMS_H12" if self.CP.carFingerprint == CAR.HYUNDAI_VENUE_2021_NON_SCC else \
                         "LVR12"
      cruise_speed_sig = "Cruise_Limit_Target" if self.CP.flags & HyundaiFlags.EV else \
                         "SLC_SET_SPEED" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "SLD_VS" if self.CP.carFingerprint == CAR.HYUNDAI_VENUE_2021_NON_SCC else \
                         "CF_Lvr_CruiseSet"
      ret.cruiseState.available = cp.vl[cruise_msg][cruise_available_sig] != 0
      ret.cruiseState.enabled = cp.vl[cruise_msg][cruise_enabled_sig] != 0
      ret.cruiseState.speed = cp.vl[cruise_speed_msg][cruise_speed_sig] * speed_conv
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False

      if not self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_NO_FCA:
        cp_cruise = cp if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_RADAR_FCA else cp_cam

        aeb_src = "FCA11"
        aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
        aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src]["FCA_CmdAct"] != 0
        ret.stockFcw = aeb_warning and not aeb_braking
        ret.stockAeb = aeb_warning and aeb_braking

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_conv

  def update_canfd_ext(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser],
                       speed_factor: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS"]["aBasis"]

    if ccnc_hud_enabled(self.CP, self.CP_SP):
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
        # the interceptor redirects the ADAS DRV ECU's copies to A-CAN once ours are seen on E-CAN
        cp_hud = can_parsers[Bus.adas]
      else:
        cp_hud = cp_cam
        self.mdps = copy.copy(cp.vl["MDPS"])
        self.cruise_info = copy.copy(cp_cam.vl["SCC_CONTROL"])
        self.cruise_buttons_msg = copy.copy(cp.vl["CRUISE_BUTTONS"])
        if self.CP_SP.flags & HyundaiFlagsSP.CANFD_LFA_ALT_STEER:
          self.lfa_alt = copy.copy(cp_cam.vl["LFA_ALT"])
      self.msg_161 = copy.copy(cp_hud.vl["CCNC_0x161"])
      self.msg_162 = copy.copy(cp_hud.vl["CCNC_0x162"])
    elif Bus.adas in can_parsers:
      self.msg_1e0 = copy.copy(can_parsers[Bus.adas].vl["LFAHDA_CLUSTER"])

    if self.CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR and self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT:
      self.lkas_alt = copy.copy(cp_cam.vl["LKAS_ALT"])

    if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS and self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
      self.cruise_buttons_msg = copy.copy(cp.vl[self.cruise_btns_msg_canfd])

    if self.CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR and self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
      self.update_dcu_emergency(ret, ret_sp, can_parsers)

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_factor

  def update_dcu_emergency(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    lfa = can_parsers[Bus.alt if dcu_lfa_on_ecan(self.CP, self.CP_SP) else Bus.adas].vl["LFA"]
    emergency = any(lfa[s] != 0 for s in ("FCA_LO_WrnSta", "FCA_LS_WrnSta", "FCA_ESA_WrnSta", "FCA_ESA_CtrlSta", "BCA_Rear_WrnSta"))
    emergency = emergency or lfa["ELK_SymbDisp"] == 3
    if self.CP_SP.flags & HyundaiFlagsSP.CANFD_ANGLE_STEERING:
      lfa_alt = can_parsers[Bus.adas].vl["LFA_ALT"]
      emergency = emergency or lfa_alt["FCA_ESA_ActvSta"] != 0 or lfa_alt["FCA_ESA_TqBstGainVal"] != 0

    # 0x160 can't tell AEB from emergency steering: both raise its stage, so only ccNC cars report stock AEB
    alert = self.msg_161.get("ALERTS_1", 0)
    ret.stockAeb = alert in DCU_EMERGENCY_BRAKING_ALERTS
    ret_sp.stockSteerIntervention = emergency or ret.stockAeb or alert in DCU_EMERGENCY_STEERING_ALERTS

  @staticmethod
  def get_parser(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> dict[StrEnum, CANParser]:
    # the interceptor redirects the ADAS DRV ECU's copies to A-CAN once ours are seen on E-CAN, and we only send after
    # canValid initializes selfdrived, so none of them can gate canValid
    acan, ecan = [], []

    if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and ccnc_hud_enabled(CP, CP_SP):
      acan += ["CCNC_0x161", "CCNC_0x162"]
    elif CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR and \
         CP.openpilotLongitudinalControl:
      # no ccNC: the ADAS DRV ECU's LFA state is in its 0x1E0
      acan.append("LFAHDA_CLUSTER")

    if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR:
      (ecan if dcu_lfa_on_ecan(CP, CP_SP) else acan).append("LFA")
      if CP_SP.flags & HyundaiFlagsSP.CANFD_ANGLE_STEERING:
        acan.append("LFA_ALT")

    messages = {}
    if acan:
      messages[Bus.adas] = CANParser(DBC[CP.carFingerprint][Bus.pt], [(m, float('nan')) for m in acan], CanBus(CP).ACAN)
    if ecan:
      messages[Bus.alt] = CANParser(DBC[CP.carFingerprint][Bus.pt], [(m, float('nan')) for m in ecan], CanBus(CP).ECAN)
    return messages


def dcu_lfa_on_ecan(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  # the ADAS DRV ECU's LFA reaches A-CAN only while we send our own
  return bool(CP_SP.flags & HyundaiFlagsSP.CANFD_ANGLE_STEERING or not CP.openpilotLongitudinalControl)
