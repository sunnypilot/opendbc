"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from opendbc.car import DT_CTRL
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
LEAD_LATERAL_MIN = -6.4  # m, 7-bit signed two's complement (-6.4..6.3 m)
LEAD_LATERAL_MAX = 6.3  # m
LEAD_LATERAL_SPAN = 12.8  # m


def encode_lead_lateral(lat_m: float) -> float:
  lat_m = round(float(np.clip(lat_m, LEAD_LATERAL_MIN, LEAD_LATERAL_MAX)), 1)
  return lat_m + LEAD_LATERAL_SPAN if lat_m < 0 else lat_m


def create_ccnc_mdps(packer, CAN, mdps):
  values = mdps.copy()

  values.update({
    "MDPS_LkaToiActvSta": 0,
    "MDPS_LkaToiUnblSta": 1,
  })

  return packer.make_can_msg("MDPS", CAN.CAM, values)


def create_ccnc_0x161(packer, CAN, msg_161, enabled, lat_active, paused, long_enabled,
                      main_cruise_enabled, gas_override, override_blink, hud_control, set_speed, target_distance, lead_visible):
  values = msg_161.copy()

  values.update({
    "LKA_ICON": 4 if lat_active else 3,
    "LFA_ICON": 2 if lat_active else 1 if paused else 0,
    "LANELINE_LEFT": 6 if lat_active else 0,
    "LANELINE_RIGHT": 6 if lat_active else 0,
    "CENTERLINE": 1 if lat_active else 0,
    "LCA_LEFT_ICON": 2 if lat_active else 1,
    "LCA_RIGHT_ICON": 2 if lat_active else 1,
    "DAW_ICON": 0,
  })

  if long_enabled:
    long_active = enabled and not gas_override
    hud_state = 2 if long_active else (3 if override_blink else 0) if enabled else 1 if main_cruise_enabled else 0
    spacing_state = 1 if long_active or (enabled and override_blink) else 0

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


def create_ccnc_0x162(packer, CAN, msg_162, enabled, gas_override, long_enabled, lead_visible, lead_distance, lead_lateral):
  values = msg_162.copy()

  if long_enabled:
    values.update({
      "LEAD": (2 if enabled and not gas_override else 1) if lead_visible else 0,
      "LEAD_DISTANCE": lead_distance if lead_visible else TARGET_DISTANCE_MAX,
      "LEAD_LATERAL": encode_lead_lateral(lead_lateral) if lead_visible else 0,
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
    self.lead_distance_filter = FirstOrderFilter(0.0, LEAD_FILTER_RC, 5 * DT_CTRL)
    self.lead_lateral_filter = FirstOrderFilter(0.0, LEAD_FILTER_RC, 5 * DT_CTRL)
    self.lead_visible_last = False

  @staticmethod
  def _get_set_speed(v_cruise_cluster, is_metric):
    unit_conv = 1 if is_metric else CV.KPH_TO_MPH
    max_speed = 145 if is_metric else 90
    min_speed = 40 if is_metric else 25
    set_speed = round(v_cruise_cluster * unit_conv)
    return min_speed if set_speed > max_speed else set_speed

  def update(self, CS, CC):
    can_sends = []

    if not self.ccnc_lfa_steering:
      return can_sends

    can_sends.append(create_ccnc_mdps(self.packer, self.CAN, CS.mdps))

    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl and self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
      press_main = CS.cruise_info["MainMode_ACC"] and self.frame % (2 * MAIN_PRESS_FRAMES) < MAIN_PRESS_FRAMES
      can_sends.append(create_ccnc_buttons(self.packer, self.CAN, CS.cruise_buttons_msg, press_main))

    if self.frame % 5 == 0:
      lat_active = self.mads.lat_active if self.mads.enable_mads else CC.enabled
      paused = self.mads.enable_mads and self.mads.paused
      gas_override = CC.cruiseControl.override
      override_blink = gas_override and self.frame % OVERRIDE_BLINK_FRAMES < OVERRIDE_BLINK_ON_FRAMES
      set_speed = self._get_set_speed(CS.out.vCruiseCluster, CS.is_metric)
      t_follow = T_FOLLOW.get(CC.hudControl.leadDistanceBars, 1.45)
      stop_distance = float(np.interp(CS.out.vEgo, STOP_DISTANCE_BP, STOP_DISTANCE_V))
      target_distance = min(TARGET_DISTANCE_MAX, max(0.0, t_follow * CS.out.vEgo + stop_distance))
      lead_visible = CC.hudControl.leadVisible and bool(self.lead_one.status)
      if lead_visible:
        if not self.lead_visible_last:
          self.lead_distance_filter.x = self.lead_distance
          self.lead_lateral_filter.x = -self.lead_one.yRel
        else:
          self.lead_distance_filter.update(self.lead_distance)
          self.lead_lateral_filter.update(-self.lead_one.yRel)
      self.lead_visible_last = lead_visible
      lead_distance = min(TARGET_DISTANCE_MAX, max(0.0, self.lead_distance_filter.x))
      can_sends.append(create_ccnc_0x161(self.packer, self.CAN, CS.msg_161, CC.enabled, lat_active, paused,
                                         self.CP.openpilotLongitudinalControl, CS.main_cruise_enabled,
                                         gas_override, override_blink, CC.hudControl, set_speed, target_distance, lead_visible))
      can_sends.append(create_ccnc_0x162(self.packer, self.CAN, CS.msg_162, CC.enabled, gas_override,
                                         self.CP.openpilotLongitudinalControl, lead_visible, lead_distance, self.lead_lateral_filter.x))

    return can_sends
