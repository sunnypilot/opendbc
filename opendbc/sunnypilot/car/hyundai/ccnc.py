"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


def ccnc_hud_enabled(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  if not CP_SP.flags & HyundaiFlagsSP.CCNC:
    return False
  if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
    # HDA2: only with openpilot long through the ADAS DRV interceptor, which blocks the ADAS DRV ECU's own copies
    return bool(CP.openpilotLongitudinalControl and CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR)
  return True


def create_ccnc(packer, CAN, openpilot_longitudinal_control, enabled, hud, left_blinker, right_blinker, msg_161, msg_162, msg_1b5,
                is_metric, out, main_cruise_enabled, lfa_icon):
  for f in {"FAULT_LSS", "FAULT_HDA", "FAULT_DAS", "FAULT_LFA", "FAULT_DAW", "FAULT_ESS"}:
    msg_162[f] = 0
  if msg_161["ALERTS_2"] == 5:
    msg_161.update({"ALERTS_2": 0, "SOUNDS_2": 0})
  if msg_161["ALERTS_3"] == 17:
    msg_161["ALERTS_3"] = 0
  if msg_161["ALERTS_5"] in (2, 5):
    msg_161["ALERTS_5"] = 0
  if msg_161["SOUNDS_4"] == 2 and msg_161["LFA_ICON"] in (0, 3):
    msg_161["SOUNDS_4"] = 0

  LANE_CHANGE_SPEED_MIN = 8.9408  # 20 mph
  any_blinker = left_blinker or right_blinker
  curvature = {i: (31 if i == -1 else 13 - abs(i + 15)) if i < 0 else 15 + i for i in range(-15, 16)}

  msg_161.update({
    "DAW_ICON": 0,
    "LKA_ICON": 0,
    "LFA_ICON": 2 if lfa_icon else 0,
    "CENTERLINE": 1 if lfa_icon else 0,
    "LANELINE_CURVATURE": curvature[max(-15, min(int(out.steeringAngleDeg / 4.5), 15))] if lfa_icon and not any_blinker else 15,
    "LANELINE_LEFT": (0 if not lfa_icon else 1 if not hud.leftLaneVisible else 4 if hud.leftLaneDepart else 6 if any_blinker else 2),
    "LANELINE_RIGHT": (0 if not lfa_icon else 1 if not hud.rightLaneVisible else 4 if hud.rightLaneDepart else 6 if any_blinker else 2),
    "LCA_LEFT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.leftBlindspot else 2 if any_blinker else 4),
    "LCA_RIGHT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.rightBlindspot else 2 if any_blinker else 4),
    "LCA_LEFT_ARROW": 2 if left_blinker else 0,
    "LCA_RIGHT_ARROW": 2 if right_blinker else 0,
  })

  if lfa_icon and any_blinker:
    left_lane_raw, right_lane_raw = msg_1b5["Info_LftLnPosVal"], msg_1b5["Info_RtLnPosVal"]

    scale_per_m = 15 / 1.7
    left_lane = abs(int(round(15 + (left_lane_raw - 1.7) * scale_per_m)))
    right_lane = abs(int(round(15 + (right_lane_raw - 1.7) * scale_per_m)))

    if msg_1b5["Info_LftLnQualSta"] not in (2, 3):
      left_lane = 0
    if msg_1b5["Info_RtLnQualSta"] not in (2, 3):
      right_lane = 0

    if left_lane_raw == -2.0248375:
      left_lane = 30 - right_lane
    if right_lane_raw == 2.0248375:
      right_lane = 30 - left_lane

    if left_lane_raw == right_lane_raw == 0:
      left_lane = right_lane = 15
    elif left_lane_raw == 0:
      left_lane = 30 - right_lane
    elif right_lane_raw == 0:
      right_lane = 30 - left_lane

    total = left_lane + right_lane
    if total == 0:
      left_lane = right_lane = 15
    else:
      left_lane = round((left_lane / total) * 30)
      right_lane = 30 - left_lane

    msg_161["LANELINE_LEFT_POSITION"] = left_lane
    msg_161["LANELINE_RIGHT_POSITION"] = right_lane

  if hud.leftLaneDepart or hud.rightLaneDepart:
    msg_162["VIBRATE"] = 1

  if openpilot_longitudinal_control:
    if msg_161["ALERTS_3"] in (1, 2, 3, 4, 7, 8, 9, 10):
      msg_161["ALERTS_3"] = 0
    if msg_161["ALERTS_5"] == 4:
      msg_161["ALERTS_5"] = 0
    if msg_161["SOUNDS_3"] == 5:
      msg_161["SOUNDS_3"] = 0

    msg_161.update({
      "SETSPEED": 3 if enabled else 1,
      "SETSPEED_HUD": 0 if not main_cruise_enabled else 2 if enabled else 1,
      "SETSPEED_SPEED": (
        255 if not main_cruise_enabled else
        (40 if is_metric else 25) if (s := round(out.vCruiseCluster * (1 if is_metric else CV.KPH_TO_MPH))) > (145 if is_metric else 90) else s
      ),
      "DISTANCE": hud.leadDistanceBars,
      "DISTANCE_SPACING": 0 if not main_cruise_enabled else 1 if enabled else 3,
      "DISTANCE_LEAD": 0 if not main_cruise_enabled else 2 if enabled and hud.leadVisible else 1 if hud.leadVisible else 0,
      "DISTANCE_CAR": 0 if not main_cruise_enabled else 2 if enabled else 1,
      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": 0,
    })

    msg_162["LEAD"] = 0 if not main_cruise_enabled else 2 if enabled else 1
    msg_162["LEAD_DISTANCE"] = msg_1b5["Longitudinal_Distance"]

  return [packer.make_can_msg(msg, CAN.ECAN, data) for msg, data in [("CCNC_0x161", msg_161), ("CCNC_0x162", msg_162)]]
