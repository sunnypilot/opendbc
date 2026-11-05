"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.crc import CRC8J1850, mk_crc8_fun

cruise_buttons_checksum = mk_crc8_fun(CRC8J1850, init_crc=0x97, xor_out=0x97)  # mk_crc8_fun xors init with xor_out: register starts at 0

MAIN_PRESS_FRAMES = 12  # 120 ms


def create_ccnc_mdps(packer, CAN, mdps):
  # report LKA torque inactive/unavailable so the camera LFA stays in standby
  return packer.make_can_msg("MDPS", CAN.CAM, {**mdps, "MDPS_LkaToiActvSta": 0, "MDPS_LkaToiUnblSta": 1})


def create_ccnc(packer, CAN, openpilot_longitudinal_control, enabled, hud, msg_161, msg_162, lead_distance, is_metric,
                v_cruise_cluster, main_cruise_enabled, lat_enabled):
  msg_161.update({
    "LKA_ICON": 4 if lat_enabled else 3,        # green lateral enabled, gray otherwise
    "LFA_ICON": 2 if lat_enabled else 1,        # green lateral enabled, gray otherwise
    "LANELINE_LEFT": 6 if lat_enabled else 0,   # green lateral enabled, gray otherwise
    "LANELINE_RIGHT": 6 if lat_enabled else 0,  # green lateral enabled, gray otherwise
    "CENTERLINE": 1 if lat_enabled else 0,      # green lateral enabled, hidden otherwise (no gray)
    "DAW_ICON": 0,
  })

  if openpilot_longitudinal_control:
    state = 0 if not main_cruise_enabled else 2 if enabled else 1               # 0 cruise main off, 1 main on, 2 engaged
    lead = 2 if enabled and hud.leadVisible else 0                              # white engaged with a lead, hidden otherwise

    msg_161.update({
      "SETSPEED": (0, 1, 3)[state],                                             # white engaged, gray main on, hidden main off
      "SETSPEED_HUD": state,                                                    # green engaged, gray main on, hidden main off
      "SETSPEED_SPEED": (
        255 if not main_cruise_enabled else
        (40 if is_metric else 25) if (s := round(v_cruise_cluster * (1 if is_metric else CV.KPH_TO_MPH))) > (145 if is_metric else 90) else s
      ),
      "DISTANCE": hud.leadDistanceBars if main_cruise_enabled else 0,
      "DISTANCE_SPACING": 1 if enabled else 0,                                  # blue gap engaged, hidden otherwise
      "DISTANCE_LEAD": lead,                                                    # lead car in gap graphic
      "DISTANCE_CAR": state,                                                    # own car white engaged, gray main on, hidden main off
      "HDA_ICON": 0,
      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": 0,
    })

    msg_162["LEAD"] = lead
    msg_162["LEAD_DISTANCE"] = lead_distance                                    # lead distance in m

  return [packer.make_can_msg(msg, CAN.ECAN, data) for msg, data in [("CCNC_0x161", msg_161), ("CCNC_0x162", msg_162)]]


def create_ccnc_buttons(packer, CAN, frame, buttons, camera_main_on):
  main_press = bool(camera_main_on) and frame % (2 * MAIN_PRESS_FRAMES) < MAIN_PRESS_FRAMES
  msgs = []
  for b in buttons:
    addr, dat, bus = packer.make_can_msg("CRUISE_BUTTONS", CAN.CAM, {**b, "ADAPTIVE_CRUISE_MAIN_BTN": int(main_press), "NORMAL_CRUISE_MAIN_BTN": 0})
    msgs.append((addr, bytes([cruise_buttons_checksum(dat[1:])]) + dat[1:], bus))
  return msgs
