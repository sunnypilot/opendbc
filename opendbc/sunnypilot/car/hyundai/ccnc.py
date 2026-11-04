from opendbc.car.common.conversions import Conversions as CV


def create_ccnc_mdps(packer, CAN, mdps):
  # report LKA torque inactive/unavailable so the camera LFA stays in standby
  return packer.make_can_msg("MDPS", CAN.CAM, {**mdps, "MDPS_LkaToiActvSta": 0, "MDPS_LkaToiUnblSta": 1})


def create_ccnc(packer, CAN, openpilot_longitudinal_control, enabled, hud, msg_161, msg_162, lead_distance, is_metric,
                v_cruise_cluster, main_cruise_enabled, lat_enabled):
  # consider taking a break, with its chime
  if msg_161["ALERTS_2"] == 5:
    msg_161.update({"ALERTS_2": 0, "SOUNDS_2": 0})

  # drive carefully
  if msg_161["ALERTS_3"] == 17:
    msg_161["ALERTS_3"] = 0

  # 2 watch for surrounding vehicles, 5 use switch or pedal to accelerate
  if msg_161["ALERTS_5"] in (2, 5):
    msg_161["ALERTS_5"] = 0

  msg_161.update({
    "LKA_ICON": 4 if lat_enabled else 3,        # green lateral enabled, gray otherwise
    "LFA_ICON": 2 if lat_enabled else 1,        # green lateral enabled, gray otherwise
    "LANELINE_LEFT": 6 if lat_enabled else 0,   # green lateral enabled, gray otherwise
    "LANELINE_RIGHT": 6 if lat_enabled else 0,  # green lateral enabled, gray otherwise
    "CENTERLINE": 1 if lat_enabled else 0,      # green lateral enabled, hidden otherwise (no gray)
    "DAW_ICON": 0,
  })

  if openpilot_longitudinal_control:
    # 1/3 speed limit, 2/4 set speed changed, 7-10 distance, 11-14 driving style
    if msg_161["ALERTS_3"] in (1, 2, 3, 4, 7, 8, 9, 10, 11, 12, 13, 14):
      msg_161["ALERTS_3"] = 0

    # smart cruise conditions not met
    if msg_161["ALERTS_5"] == 4:
      msg_161["ALERTS_5"] = 0

    # single chime
    if msg_161["SOUNDS_3"] == 5:
      msg_161["SOUNDS_3"] = 0

    state = 0 if not main_cruise_enabled else 2 if enabled else 1               # 0 cruise main off, 1 main on, 2 engaged
    lead = 2 if enabled and hud.leadVisible else 0                              # white engaged with a lead, hidden otherwise

    msg_161.update({
      "SETSPEED": (0, 1, 3)[state],                                             # white engaged, gray main on, hidden main off
      "SETSPEED_HUD": state,                                                    # green engaged, gray main on, hidden main off
      "SETSPEED_SPEED": (
        255 if not main_cruise_enabled else
        (40 if is_metric else 25) if (s := round(v_cruise_cluster * (1 if is_metric else CV.KPH_TO_MPH))) > (145 if is_metric else 90) else s
      ),
      "DISTANCE": hud.leadDistanceBars if main_cruise_enabled else 0,           # distance bars, hidden main off
      "DISTANCE_SPACING": 1 if enabled else 0,                                  # blue gap engaged, hidden otherwise
      "DISTANCE_LEAD": lead,                                                    # lead car in gap graphic
      "DISTANCE_CAR": state,                                                    # own car white engaged, gray main on, hidden main off
      "HDA_ICON": 0,
      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": 0,
    })

    msg_162["LEAD"] = lead                                                      # lead box
    msg_162["LEAD_DISTANCE"] = lead_distance                                    # lead distance in m

  return [packer.make_can_msg(msg, CAN.ECAN, data) for msg, data in [("CCNC_0x161", msg_161), ("CCNC_0x162", msg_162)]]
