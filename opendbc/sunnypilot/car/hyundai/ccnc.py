from opendbc.car.common.conversions import Conversions as CV


def create_ccnc_mdps(packer, CAN, mdps):
  # report LKA torque inactive/unavailable so the camera LFA stays in standby
  return packer.make_can_msg("MDPS", CAN.CAM, {**mdps, "MDPS_LkaToiActvSta": 0, "MDPS_LkaToiUnblSta": 1})


def create_ccnc(packer, CAN, openpilot_longitudinal_control, enabled, hud, left_blinker, right_blinker, msg_161, msg_162, lead_distance,
                is_metric, out, main_cruise_enabled, lat_active, lkas_icon, lfa_icon, CC_SP, frame):
  # consider taking a break, with its chime
  if msg_161["ALERTS_2"] == 5:
    msg_161.update({"ALERTS_2": 0, "SOUNDS_2": 0})
  # drive carefully
  if msg_161["ALERTS_3"] == 17:
    msg_161["ALERTS_3"] = 0
  # 2 watch for surrounding vehicles, 5 use switch or pedal to accelerate
  if msg_161["ALERTS_5"] in (2, 5):
    msg_161["ALERTS_5"] = 0
  # double chime while LFA icon is hidden or white
  if msg_161["SOUNDS_4"] == 2 and msg_161["LFA_ICON"] in (0, 3):
    msg_161["SOUNDS_4"] = 0

  scc, sl, icbm, lane_change = CC_SP.smartCruiseControl, CC_SP.speedLimit, CC_SP.intelligentCruiseButtonManagement, CC_SP.laneChange

  LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS  # below this lane change assist is in standby
  FOLLOW_TIME = (1.25, 1.45, 1.75)           # openpilot follow time in seconds for distance bars 1-3

  blink = frame % 100 < 60      # stock blink for most icons: 0.6 s on, 0.4 s off
  blink_fast = frame % 50 < 30  # stock LFA disengage blink: 0.3 s on, 0.2 s off
  override = out.gasPressed     # driver on the gas while engaged, icons blink

  lane_change_standby = out.vEgo < LANE_CHANGE_SPEED_MIN
  # lane change in progress: blinker on, lane change assist ready, and that side's blind spot is clear
  changing_left = lat_active and enabled and not lane_change_standby and left_blinker and not out.leftBlindspot
  changing_right = lat_active and enabled and not lane_change_standby and right_blinker and not out.rightBlindspot
  changing = changing_left or changing_right
  # lane change assist ready: steering, SCC on, and above 20 mph
  lca_ready = main_cruise_enabled and lat_active and enabled and not lane_change_standby
  # lane change assist icon, stock sends the same value to both sides
  lca_icon = (0 if not (main_cruise_enabled and lat_active) else  # hidden: cruise main off or not steering
              1 if not lca_ready else                             # gray: standby, SCC canceled or below 20 mph
              (2 if blink else 0) if changing else                # green blinking: changing lanes
              2)                                                  # green: ready

  msg_161.update({
    "DAW_ICON": 0,  # hidden: no coffee cup, openpilot handles driver monitoring

    "LKA_ICON": (0, 3, 4, 3)[lkas_icon],  # MADS 0 hidden, 1 available = 3 gray, 2 steering = 4 green, 3 disengaging = 3 gray

    "LFA_ICON": (0, 1, 2, 3 if blink_fast else 0)[lfa_icon],  # 0 hidden, 1 gray standby, 2 green steering, 3/0 white fast blink on disengage

    "HDA_ICON": (0 if not (main_cruise_enabled and lat_active) else  # hidden: cruise main off or not steering
                 1 if not enabled else                               # gray: SCC canceled, steering still on
                 (3 if blink else 0) if override else                # white blinking: gas override
                 2),                                                 # green: SCC and steering both on

    "CENTERLINE": 1 if lat_active else 0,  # green center path while steering, hidden otherwise

    "LANELINE_LEFT": (0 if not lat_active or not hud.leftLaneVisible else  # gray: not steering or line not seen
                      4 if hud.leftLaneDepart else                         # orange: departing over this line
                      (6 if blink else 2) if changing else                 # green/white blinking: changing lanes, both lines blink
                      6 if lca_ready and not out.leftBlindspot else        # green: lane change possible to this side
                      2),                                                  # white: line seen, lane change not possible to this side

    "LANELINE_RIGHT": (0 if not lat_active or not hud.rightLaneVisible else  # gray: not steering or line not seen
                       4 if hud.rightLaneDepart else                         # orange: departing over this line
                       (6 if blink else 2) if changing else                  # green/white blinking: changing lanes, both lines blink
                       6 if lca_ready and not out.rightBlindspot else        # green: lane change possible to this side
                       2),                                                   # white: line seen, lane change not possible to this side

    "LCA_LEFT_ICON": lca_icon,  # lane change assist icon, same as right

    "LCA_RIGHT_ICON": lca_icon,  # lane change assist icon, same as left

    "LCA_LEFT_ARROW": 2 if changing_left else 0,  # left arrow only while changing lanes left

    "LCA_RIGHT_ARROW": 2 if changing_right else 0,  # right arrow only while changing lanes right

    "LANE_LEFT": 1 if lane_change.state == "laneChangeStarting" and lane_change.direction == "left" else 0,  # green target lane while moving left

    "LANE_RIGHT": 1 if lane_change.state == "laneChangeStarting" and lane_change.direction == "right" else 0,  # green target lane while moving right

    "LANE_HIGHLIGHT": 1 if lane_change.state == "laneChangeFinishing" else 0,  # green new lane after crossing the line
  })

  if hud.leftLaneDepart or hud.rightLaneDepart:
    msg_162["VIBRATE"] = 1  # steering wheel vibration on lane departure

  if openpilot_longitudinal_control:
    # 1/3 adjusting to speed limit, 2/4 set speed changed
    if msg_161["ALERTS_3"] in (1, 2, 3, 4):
      msg_161["ALERTS_3"] = 0
    # the camera still pops up the distance on the distance button (7-10 distance 1-4, 11-14 with driving style)
    # keep its timing but show the openpilot distance, 7-9 distance 1-3
    if 7 <= msg_161["ALERTS_3"] <= 14:
      msg_161["ALERTS_3"] = 6 + hud.leadDistanceBars
    # smart cruise conditions not met
    if msg_161["ALERTS_5"] == 4:
      msg_161["ALERTS_5"] = 0
    # single chime
    if msg_161["SOUNDS_3"] == 5:
      msg_161["SOUNDS_3"] = 0

    scc_on = main_cruise_enabled and enabled
    scc_states = {scc.visionState, scc.mapState}
    # set speed in cluster units, above the cluster max (145 km/h, 90 mph) shows the minimum
    set_speed = round(out.vCruiseCluster * (1 if is_metric else CV.KPH_TO_MPH))
    set_speed = (40 if is_metric else 25) if set_speed > (145 if is_metric else 90) else set_speed
    # speed limit at or above the set speed, SLA arrow points up
    sla_up = round(sl.speedLimitFinalLast * CV.MS_TO_KPH) >= round(out.vCruiseCluster)
    # target gap in m: speed times the openpilot follow time for the distance bars
    target_gap = out.vEgo * FOLLOW_TIME[min(max(hud.leadDistanceBars, 1), 3) - 1]

    msg_161.update({
      "SETSPEED": (0 if not main_cruise_enabled else     # hidden: cruise main off
                   1 if not enabled else                 # gray: SCC canceled
                   (3 if blink else 0) if override else  # white blinking: gas override
                   3),                                   # white: SCC on

      "SETSPEED_HUD": (0 if not main_cruise_enabled else     # hidden: cruise main off
                       1 if not enabled else                 # gray: SCC canceled
                       (3 if blink else 0) if override else  # white blinking: gas override
                       2),                                   # green: SCC on

      "SETSPEED_SPEED": 255 if not main_cruise_enabled else set_speed,  # set speed, 255 blank when cruise main off

      "DISTANCE": hud.leadDistanceBars if main_cruise_enabled else 0,  # distance bars 1-3, hidden when cruise main off

      # gap graphic: blue is normal distance (popups 7-10), white is SCC machine learning "Based on Driving Style" (popups 11-14)
      # openpilot has no driving style mode, so always blue
      "DISTANCE_SPACING": (0 if not scc_on else                  # hidden: SCC off or canceled
                           (1 if blink else 0) if override else  # blue blinking: gas override
                           1),                                   # blue: SCC on

      "DISTANCE_LEAD": (0 if not main_cruise_enabled or not hud.leadVisible else  # hidden: cruise main off or no lead
                        2 if enabled else                                         # white: lead followed by SCC
                        1),                                                       # gray: lead seen, SCC canceled

      "DISTANCE_CAR": 0 if not main_cruise_enabled else 2 if enabled else 1,  # own car in gap graphic: white SCC on, gray canceled

      "SLA_ICON": (0 if sl.assistState not in ("preActive", "pending", "adapting", "active") else  # hidden: speed limit assist off
                   (1 if sla_up else 2) if sl.assistState in ("preActive", "pending") else         # white up/down: waiting to confirm
                   4 if sl.assistState == "adapting" else                                          # green down: slowing to the limit
                   3),                                                                             # green up: holding the limit

      "NAV_ICON": ((2 if blink else 0) if {"entering", "turning"} & scc_states else  # green blinking: slowing for a curve
                   (4 if blink else 0) if "overriding" in scc_states else            # white blinking: gas override
                   2 if {"enabled", "leaving"} & scc_states else                     # green: ready
                   0),                                                               # hidden: vision and map SCC off

      "TARGET": 0 if not scc_on else (1 if blink else 0) if override else 1,  # target gap marker, same as DISTANCE_SPACING

      "TARGET_DISTANCE": target_gap if scc_on else 204.6,  # target gap in m, 204.6 when off
    })

    msg_162["LEAD"] = (2 if enabled else 1) if main_cruise_enabled and hud.leadVisible else 0  # lead box: white followed, gray seen
    msg_162["LEAD_DISTANCE"] = lead_distance                                                   # lead distance in m

  # stock long: show ICBM set speed changes, otherwise keep the stock icon (vTarget is in cluster units)
  elif icbm.state in ("preActive", "increasing", "decreasing"):
    msg_161["SLA_ICON"] = (3 if icbm.state == "increasing" else  # green up: ICBM pressing +
                           4 if icbm.state == "decreasing" else  # green down: ICBM pressing -
                           # white up/down: ICBM about to start, arrow points from set speed toward the target
                           1 if icbm.vTarget >= round(out.vCruiseCluster * (1 if is_metric else CV.KPH_TO_MPH)) else 2)

  return [packer.make_can_msg(msg, CAN.ECAN, data) for msg, data in [("CCNC_0x161", msg_161), ("CCNC_0x162", msg_162)]]
