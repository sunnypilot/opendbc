"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from opendbc.car import DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

cruise_buttons_checksum = mk_crc8_fun(CRC8J1850, init_crc=0x97, xor_out=0x97)  # mk_crc8_fun xors init with xor_out: register starts at 0

MAIN_PRESS_FRAMES = 12  # 120 ms
OVERRIDE_BLINK_FRAMES = 100
OVERRIDE_BLINK_ON_FRAMES = 60
STOP_DISTANCE_BP = [0.0, 5.0]  # m/s
STOP_DISTANCE_V = [3.5, 6.0]  # m
T_FOLLOW = {1: 1.25, 2: 1.45, 3: 1.75}  # s
TARGET_DISTANCE_MAX = 204.6  # m
LEAD_FILTER_RC = 0.5  # s


# the interceptor firmware toggles the DCU's main with injected presses; the DCU's cruise popups would show
DCU_ALERTS_2_KEEP = (7, 8, 9, 10, 14, 21)


def drop_dcu_cruise_alerts(values: dict) -> None:
  if values["ALERTS_2"] not in DCU_ALERTS_2_KEEP and values["ALERTS_2"]:
    values["ALERTS_2"] = 0
    values["SOUNDS_2"] = 0
  if values["ALERTS_3"] or values["ALERTS_4"] or values["ALERTS_5"]:
    values.update({"ALERTS_3": 0, "ALERTS_4": 0, "ALERTS_5": 0, "SOUNDS_3": 0, "SOUNDS_4": 0})


def ccnc_hud_enabled(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  if not CP_SP.flags & HyundaiFlagsSP.CCNC:
    return False
  if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
    # HDA2: only through the ADAS DRV interceptor, which blocks the ADAS DRV ECU's own copies. Needed without
    # openpilot long on angle cars: we steer the MDPS directly, so the ADAS DRV ECU's LFA icon stays gray
    return bool(CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR and
                (CP.openpilotLongitudinalControl or CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING))
  return True


def create_ccnc_mdps(packer, CAN, mdps):
  values = mdps.copy()

  values.update({
    "MDPS_LkaToiActvSta": 0,
    "MDPS_LkaToiUnblSta": 1,
  })

  return packer.make_can_msg("MDPS", CAN.CAM, values)


def create_ccnc_0x161(packer, CAN, frame, msg_161, enabled, lat_active, paused, long_enabled, main_cruise_enabled, gas_override,
                      hud_control, set_speed, v_ego, lead_visible, show_distance_bars, interceptor=False):
  values = msg_161.copy()

  values.update({
    "LKA_ICON": 4 if lat_active else 3,
    "LFA_ICON": 2 if lat_active else 1 if paused else 0,
    "LANELINE_LEFT": 6 if lat_active else 0,
    "LANELINE_RIGHT": 6 if lat_active else 0,
    "CENTERLINE": 1 if lat_active else 0,
    "LCA_LEFT_ICON": 2 if lat_active else 0,
    "LCA_RIGHT_ICON": 2 if lat_active else 0,
    "DAW_ICON": 0,
  })

  # suppress consider taking a break and its chime
  if values["ALERTS_2"] == 5:
    values.update({
      "ALERTS_2": 0,
      "SOUNDS_2": 0,
    })

  if long_enabled:
    if interceptor:
      drop_dcu_cruise_alerts(values)
    override_blink = gas_override and frame % OVERRIDE_BLINK_FRAMES < OVERRIDE_BLINK_ON_FRAMES
    long_active = enabled and not gas_override
    hud_state = 2 if long_active else (3 if override_blink else 0) if enabled else 1 if main_cruise_enabled else 0
    spacing_state = 1 if long_active or (enabled and override_blink) else 0

    t_follow = T_FOLLOW.get(hud_control.leadDistanceBars, 1.45)
    stop_distance = float(np.interp(v_ego, STOP_DISTANCE_BP, STOP_DISTANCE_V))
    target_distance = min(TARGET_DISTANCE_MAX, max(0.0, t_follow * v_ego + stop_distance))

    if enabled and show_distance_bars and not values["ALERTS_3"]:
      values["ALERTS_3"] = 6 + hud_control.leadDistanceBars

    values.update({
      "SETSPEED": hud_state,
      "SETSPEED_HUD": hud_state,
      "SETSPEED_SPEED": set_speed if main_cruise_enabled else 255,
      "DISTANCE": hud_control.leadDistanceBars if enabled else 4 if main_cruise_enabled else 0,
      "DISTANCE_SPACING": spacing_state,
      "DISTANCE_LEAD": (2 if long_active else 1 if main_cruise_enabled else 0) if lead_visible else 0,
      "DISTANCE_CAR": 2 if enabled else 1 if main_cruise_enabled else 0,
      "HDA_ICON": hud_state,
      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": spacing_state,
      "TARGET_DISTANCE": target_distance if enabled else TARGET_DISTANCE_MAX,
    })

  return packer.make_can_msg("CCNC_0x161", CAN.ECAN, values)


def create_ccnc_0x162(packer, CAN, msg_162, enabled, long_enabled, main_cruise_enabled, gas_override, lead_visible, lead_distance):
  values = msg_162.copy()

  if long_enabled:
    values.update({
      "LEAD": (2 if enabled and not gas_override else 1 if main_cruise_enabled else 0) if lead_visible else 0,
      "LEAD_DISTANCE": min(TARGET_DISTANCE_MAX, max(0.0, lead_distance)) if lead_visible else TARGET_DISTANCE_MAX,
      "LEAD_LATERAL": 0,
    })

  return packer.make_can_msg("CCNC_0x162", CAN.ECAN, values)


def create_ccnc_buttons(packer, CAN, cruise_buttons_msg, main_btn):
  values = cruise_buttons_msg.copy()

  values.update({
    "ADAPTIVE_CRUISE_MAIN_BTN": 1 if main_btn else 0,
    "NORMAL_CRUISE_MAIN_BTN": 0,
  })

  dat = packer.make_can_msg("CRUISE_BUTTONS", CAN.CAM, values)[1]
  values["_CHECKSUM"] = cruise_buttons_checksum(dat[1:])

  return packer.make_can_msg("CRUISE_BUTTONS", CAN.CAM, values)


class CcncCarController:
  def __init__(self, CP, CP_SP):
    self.ccnc_lfa_steering = bool(CP_SP.flags & HyundaiFlagsSP.CCNC and not (CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG))
    self.ccnc_hud = ccnc_hud_enabled(CP, CP_SP)
    self.ccnc_interceptor = bool(CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR)
    self.lead_distance_filter = FirstOrderFilter(TARGET_DISTANCE_MAX, LEAD_FILTER_RC, 5 * DT_CTRL, initialized=False)
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0

  @staticmethod
  def _get_set_speed(v_cruise_cluster, is_metric):
    unit_conv = 1 if is_metric else CV.KPH_TO_MPH
    max_speed = 145 if is_metric else 90
    min_speed = 40 if is_metric else 25
    set_speed = round(v_cruise_cluster * unit_conv)
    return min_speed if set_speed > max_speed else set_speed

  def update(self, CS, CC):
    can_sends = []

    if not self.ccnc_hud:
      return can_sends

    if self.ccnc_lfa_steering:
      can_sends.append(create_ccnc_mdps(self.packer, self.CAN, CS.mdps))

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl and self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
        press_main = CS.cruise_info["MainMode_ACC"] and self.frame % (2 * MAIN_PRESS_FRAMES) < MAIN_PRESS_FRAMES
        can_sends.append(create_ccnc_buttons(self.packer, self.CAN, CS.cruise_buttons_msg, press_main))

    if self.CP.openpilotLongitudinalControl:
      if not CC.enabled or CC.hudControl.leadDistanceBars != self.lead_distance_bars_last:
        self.distance_bar_frame = self.frame
      self.lead_distance_bars_last = CC.hudControl.leadDistanceBars

      if self.frame % 5 == 0:
        if not self.lead_visible:
          self.lead_distance_filter.initialized = False
        elif self.lead_one.status:
          self.lead_distance_filter.update(self.lead_distance)

    if self.frame % 5 == 0:
      lat_active = self.mads.lat_active if self.mads.enable_mads else CC.enabled
      paused = self.mads.enable_mads and self.mads.paused
      set_speed = self._get_set_speed(CS.out.vCruiseCluster, CS.is_metric)

      can_sends.append(create_ccnc_0x161(self.packer, self.CAN, self.frame, CS.msg_161, CC.enabled, lat_active, paused,
                                         self.CP.openpilotLongitudinalControl, CS.main_cruise_enabled, CC.cruiseControl.override,
                                         CC.hudControl, set_speed, CS.out.vEgo, self.lead_visible,
                                         self.frame - self.distance_bar_frame < 400, self.ccnc_interceptor))
      can_sends.append(create_ccnc_0x162(self.packer, self.CAN, CS.msg_162, CC.enabled, self.CP.openpilotLongitudinalControl,
                                         CS.main_cruise_enabled, CC.cruiseControl.override, self.lead_visible,
                                         self.lead_distance_filter.x))

    return can_sends
