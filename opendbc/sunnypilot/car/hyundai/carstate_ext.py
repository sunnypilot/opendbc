"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import copy
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.dbc import SignalType
from opendbc.can.parser import CANParser
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, CAR, DBC
from opendbc.sunnypilot.car.hyundai.ccnc import ccnc_hud_enabled
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

DCU_STEER_TIMEOUT_FRAMES = 3  # 3 periods of its steering frame

DCU_FCA_WARNINGS = ("FCA_LO_WrnSta", "FCA_LS_WrnSta", "FCA_ESA_WrnSta", "FCA_ESA_CtrlSta", "BCA_Rear_WrnSta")
DCU_EMERGENCY_SIGNALS = {
  "LFA": (*DCU_FCA_WARNINGS, "ELK_SymbDisp", "ActToiSta"),
  "LFA_ALT": ("FCA_ESA_ActvSta", "FCA_ESA_TqBstGainVal", "ADAS_ActvACILvl2Sta"),
  "ADRV_0x160": ("AEB_BRAKE_REQ",),
}


def decode_only(cp: CANParser, signals: dict[str, tuple[str, ...]]) -> CANParser:
  # the parser decodes every signal each frame, and these wide CAN FD frames are read for a few. Counter and checksum stay checked
  for state in cp.message_states.values():
    if state.name in signals:
      state.signals = [s for s in state.signals if s.name in signals[state.name] or s.type != SignalType.DEFAULT]
  return cp


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
    self.lkas = {}
    self.lfa_alt = {}
    self.dcu_intercepted = bool(CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR and CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG)
    self.dcu_angle = bool(CP_SP.flags & HyundaiFlagsSP.CANFD_ANGLE_STEERING)
    self.dcu_steer_msg = "LFA_ALT" if self.dcu_angle else "LFA"
    self.dcu_steer_age = DCU_STEER_TIMEOUT_FRAMES
    self.dcu_passthrough = False

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

    if self.dcu_intercepted:
      self.lkas = copy.copy(cp_cam.vl["LKAS_ALT" if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else "LKAS"])

    if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS and self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
      self.cruise_buttons_msg = copy.copy(cp.vl[self.cruise_btns_msg_canfd])

    if self.dcu_intercepted:
      self.update_dcu_emergency(ret, ret_sp, can_parsers)

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_factor

  def dcu_steer_frame(self, can_parsers: dict[StrEnum, CANParser]) -> tuple[dict[str, float], bool]:
    # the ADAS DRV's steering frame is on A-CAN while the interceptor diverts it, and on E-CAN while it reaches the car
    cp = max(can_parsers[Bus.adas], can_parsers[Bus.alt], key=lambda p: p.ts_nanos[self.dcu_steer_msg]["COUNTER"])
    return cp.vl[self.dcu_steer_msg], len(cp.vl_all[self.dcu_steer_msg]["COUNTER"]) > 0

  def update_dcu_emergency(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    steer, fresh = self.dcu_steer_frame(can_parsers)
    # angle cars: its LFA is never diverted, so the warnings are always on E-CAN
    lfa = can_parsers[Bus.alt].vl["LFA"] if self.dcu_angle else steer

    warning = any(lfa[s] != 0 for s in DCU_FCA_WARNINGS) or lfa["ELK_SymbDisp"] == 3
    if self.dcu_angle:
      esa = steer["FCA_ESA_ActvSta"] != 0 or steer["FCA_ESA_TqBstGainVal"] != 0
      dcu_steering = esa or steer["ADAS_ActvACILvl2Sta"] == 2
    else:
      esa = False
      dcu_steering = steer["ActToiSta"] != 0

    ret_sp.stockSteerIntervention = warning or esa
    # the 0x160 stage also rises during emergency steering, only a brake request is AEB
    ret.stockAeb = can_parsers[Bus.alt].vl["ADRV_0x160"]["AEB_BRAKE_REQ"] > 0

    # while it steers in an emergency we hold our steering frame back, and the interceptor lets its own reach the car
    self.dcu_steer_age = 0 if fresh else self.dcu_steer_age + 1
    self.dcu_passthrough = ret_sp.stockSteerIntervention and dcu_steering and self.dcu_steer_age < DCU_STEER_TIMEOUT_FRAMES

  @staticmethod
  def get_parser(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> dict[StrEnum, CANParser]:
    # the interceptor redirects the ADAS DRV ECU's copies to A-CAN once ours are seen on E-CAN, and we only send after
    # canValid initializes selfdrived, so none of them can gate canValid
    acan, ecan = [], []

    if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and ccnc_hud_enabled(CP, CP_SP):
      acan += ["CCNC_0x161", "CCNC_0x162"]
    elif CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR:
      # no ccNC: the ADAS DRV ECU's LFA state is in its 0x1E0
      acan.append("LFAHDA_CLUSTER")

    if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR:
      # the ADAS DRV's steering frame, blocked or not. Its LFA is never blocked on angle cars
      steer_msg = "LFA_ALT" if CP_SP.flags & HyundaiFlagsSP.CANFD_ANGLE_STEERING else "LFA"
      acan.append(steer_msg)
      ecan += list(dict.fromkeys([steer_msg, "LFA", "ADRV_0x160"]))

    messages = {}
    if acan:
      messages[Bus.adas] = decode_only(CANParser(DBC[CP.carFingerprint][Bus.pt], [(m, float('nan')) for m in acan], CanBus(CP).ACAN),
                                       DCU_EMERGENCY_SIGNALS)
    if ecan:
      messages[Bus.alt] = decode_only(CANParser(DBC[CP.carFingerprint][Bus.pt], [(m, float('nan')) for m in ecan], CanBus(CP).ECAN),
                                      DCU_EMERGENCY_SIGNALS)
    return messages
