"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

cruise_buttons_checksum = mk_crc8_fun(CRC8J1850, init_crc=0x97, xor_out=0x97)  # mk_crc8_fun xors init with xor_out: register starts at 0

MAIN_PRESS_FRAMES = 12  # 120 ms


def create_ccnc_mdps(packer, CAN, mdps):
  values = mdps.copy()

  values.update({
    "MDPS_LkaToiActvSta": 0,
    "MDPS_LkaToiUnblSta": 1,
  })

  return packer.make_can_msg("MDPS", CAN.CAM, values)


def create_ccnc(packer, CAN, openpilot_longitudinal_control, enabled, mads, hud, msg_161, msg_162, lead_distance,
                is_metric, v_cruise_cluster, main_cruise_enabled):
  lat_active = mads.lat_active if mads.enable_mads else enabled
  paused = mads.enable_mads and mads.paused

  values_161 = msg_161.copy()
  values_161.update({
    "LKA_ICON": 4 if lat_active else 3,
    "LFA_ICON": 2 if lat_active else 1 if paused else 0,
    "LANELINE_LEFT": 6 if lat_active else 0,
    "LANELINE_RIGHT": 6 if lat_active else 0,
    "CENTERLINE": 1 if lat_active else 0,
    "DAW_ICON": 0,
  })

  values_162 = msg_162.copy()

  if openpilot_longitudinal_control:
    state = 0 if not main_cruise_enabled else 2 if enabled else 1
    lead = 2 if enabled and hud.leadVisible else 0

    values_161.update({
      "SETSPEED": (0, 1, 3)[state],
      "SETSPEED_HUD": state,
      "SETSPEED_SPEED": (
        255 if not main_cruise_enabled else
        (40 if is_metric else 25) if (s := round(v_cruise_cluster * (1 if is_metric else CV.KPH_TO_MPH))) > (145 if is_metric else 90) else s
      ),
      "DISTANCE": hud.leadDistanceBars if main_cruise_enabled else 0,
      "DISTANCE_SPACING": 1 if enabled else 0,
      "DISTANCE_LEAD": lead,
      "DISTANCE_CAR": state,
      "HDA_ICON": 0,
      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": 0,
    })

    values_162["LEAD"] = lead
    values_162["LEAD_DISTANCE"] = lead_distance

  return [
    packer.make_can_msg("CCNC_0x161", CAN.ECAN, values_161),
    packer.make_can_msg("CCNC_0x162", CAN.ECAN, values_162),
  ]


def create_ccnc_buttons(packer, CAN, frame, buttons, camera_main_on):
  main_press = bool(camera_main_on) and frame % (2 * MAIN_PRESS_FRAMES) < MAIN_PRESS_FRAMES
  msgs = []
  for b in buttons:
    addr, dat, bus = packer.make_can_msg("CRUISE_BUTTONS", CAN.CAM, {**b, "ADAPTIVE_CRUISE_MAIN_BTN": int(main_press), "NORMAL_CRUISE_MAIN_BTN": 0})
    msgs.append((addr, bytes([cruise_buttons_checksum(dat[1:])]) + dat[1:], bus))
  return msgs


class CcncCarController:
  def __init__(self, CP, CP_SP):
    self.ccnc_lfa_steering = bool(CP_SP.flags & HyundaiFlagsSP.CCNC and not (CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG))

  def update(self, CS, CC):
    can_sends = []

    if not self.ccnc_lfa_steering:
      return can_sends

    can_sends.append(create_ccnc_mdps(self.packer, self.CAN, CS.mdps))
    if self.CP.openpilotLongitudinalControl and self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
      can_sends.extend(create_ccnc_buttons(self.packer, self.CAN, self.frame, CS.ccnc_buttons, CS.cruise_info["MainMode_ACC"]))
    if self.frame % 5 == 0:
      can_sends.extend(create_ccnc(self.packer, self.CAN, self.CP.openpilotLongitudinalControl, CC.enabled, self.mads, CC.hudControl,
                                   CS.msg_161, CS.msg_162, self.lead_distance, CS.is_metric, CS.out.vCruiseCluster, CS.main_cruise_enabled))
    return can_sends
