from opendbc.car import structs

VisualAlert = structs.CarControl.HUDControl.VisualAlert


def create_ccnc_mdps(packer, CAN, mdps):
  # report LKA torque inactive/unavailable so the camera LFA stays in standby
  return packer.make_can_msg("MDPS", CAN.CAM, {**mdps, "MDPS_LkaToiActvSta": 0, "MDPS_LkaToiUnblSta": 1})


def create_ccnc(packer, CAN, openpilot_longitudinal_control, enabled, hud, msg_161, msg_162, lead_distance, set_speed,
                main_cruise_enabled, lat_active, lkas_icon, lfa_icon):
  # cluster UI like Hyundai CAN LKAS11/LFAHDA_MFC/SCC11: icons, lane lines, lane departure, hands-on-wheel, set speed, gap, lead

  # stock camera nags: consider taking a break (with chime), drive carefully, watch for surrounding vehicles, use switch or pedal
  if msg_161["ALERTS_2"] == 5:
    msg_161.update({"ALERTS_2": 0, "SOUNDS_2": 0})
  if msg_161["ALERTS_3"] == 17:
    msg_161["ALERTS_3"] = 0
  if msg_161["ALERTS_5"] in (2, 5):
    msg_161["ALERTS_5"] = 0

  def laneline(visible, depart):
    return 4 if depart else 6 if visible and enabled else 2 if visible else 0  # orange departing, green engaged, white seen, gray

  msg_161.update({
    "LKA_ICON": (0, 3, 4, 3)[lkas_icon],  # 3 gray, 4 green; DBC max is 3 but 4 is green
    "LFA_ICON": lfa_icon,
    "DAW_ICON": 0,
    "CENTERLINE": 1 if lat_active else 0,
    "LANELINE_LEFT": laneline(hud.leftLaneVisible, hud.leftLaneDepart),
    "LANELINE_RIGHT": laneline(hud.rightLaneVisible, hud.rightLaneDepart),
  })
  if hud.visualAlert == VisualAlert.steerRequired:
    msg_161["ALERTS_2"] = 1  # keep hands on steering wheel
  if hud.leftLaneDepart or hud.rightLaneDepart:
    msg_162["VIBRATE"] = 1

  if openpilot_longitudinal_control:
    # stock SCC alerts that don't apply: adjusting to speed limit / set speed changed, SCC conditions not met, single chime
    if msg_161["ALERTS_3"] in (1, 2, 3, 4):
      msg_161["ALERTS_3"] = 0
    if msg_161["ALERTS_5"] == 4:
      msg_161["ALERTS_5"] = 0
    if msg_161["SOUNDS_3"] == 5:
      msg_161["SOUNDS_3"] = 0

    # camera still pops up the distance on the button (7-10, 11-14 driving style), show openpilot's: 7-9 distance 1-3
    if 7 <= msg_161["ALERTS_3"] <= 14:
      msg_161["ALERTS_3"] = 6 + hud.leadDistanceBars

    state = 0 if not main_cruise_enabled else 2 if enabled else 1  # hidden, gray standby, active
    lead = 2 if enabled else 1
    msg_161.update({
      "HDA_ICON": state,
      "SETSPEED": (0, 1, 3)[state],
      "SETSPEED_HUD": state,
      "SETSPEED_SPEED": set_speed if main_cruise_enabled else 255,
      "DISTANCE": hud.leadDistanceBars if main_cruise_enabled else 0,
      "DISTANCE_SPACING": 1 if enabled else 0,
      "DISTANCE_LEAD": lead if main_cruise_enabled and hud.leadVisible else 0,
      "DISTANCE_CAR": state,
      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": 0,
    })
    msg_162["LEAD"] = lead if main_cruise_enabled and hud.leadVisible else 0
    msg_162["LEAD_DISTANCE"] = lead_distance

  return [packer.make_can_msg(msg, CAN.ECAN, data) for msg, data in [("CCNC_0x161", msg_161), ("CCNC_0x162", msg_162)]]
