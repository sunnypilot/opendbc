from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV


LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS  # keep in sync with openpilot desire_helper.LANE_CHANGE_SPEED_MIN
LANE_POSITION_GAIN = 12                    # stock LANELINE_*_POSITION steps per m of lane offset
LANE_HALF_WIDTH = 1.75                     # m, lane center from a single line
DISTANCE_HOLD_TIME = 0.2                   # s, a new distance setting has to hold this long before it shows, hides openpilot's brief flip-back
SLA_CONFIRMED_SHOW_TIME = 2.0              # s, the green speed limit arrow shows this long after confirming, then hides
LANE_CROSS_JUMP = 1.0                      # m, the camera lane offset jumps about 2 m when it relabels the lanes at the crossing

# remembered across calls: whether the current lane change crossed the line yet, and the last lane offset
lane_change_progress = {"crossed": False, "offset": None}

# remembered across calls: the speed limit arrow shown (1/2 white up/down asking, 3/4 green up/down confirmed) and the frame it was set
sla_progress = {"arrow": 0, "frame": 0}

# remembered across calls: the distance bars shown, and a new value waiting to hold long enough
distance_progress = {"shown": None, "pending": None, "frame": 0}


def create_ccnc_mdps(packer, CAN, mdps):
  # report LKA torque inactive/unavailable so the camera LFA stays in standby
  return packer.make_can_msg("MDPS", CAN.CAM, {**mdps, "MDPS_LkaToiActvSta": 0, "MDPS_LkaToiUnblSta": 1})


def create_ccnc(packer, CAN, openpilot_longitudinal_control, enabled, hud, left_blinker, right_blinker, msg_161, msg_162, msg_1b5, lead_distance,
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
  dm, lead_following = CC_SP.driverMonitoring, CC_SP.leadFollowing

  # openpilot driver monitoring alert in the stock HDP style
  if dm.alertLevel == 3:
    msg_161["ALERTS_2"] = 9  # take control of the vehicle immediately
  elif dm.alertLevel > 0:
    msg_161["ALERTS_2"] = (12 if dm.visionPolicy else    # keep your eyes on the road, driver camera sees the face
                           1 if dm.alertLevel == 1 else  # keep hands on steering wheel, no face seen
                           2)                            # keep hands on steering wheel in red, no face seen

  blink = frame % 100 < 60      # stock blink for most icons: 0.6 s on, 0.4 s off
  blink_fast = frame % 50 < 30  # stock LFA disengage and LKA departure blink: 0.3 s on, 0.2 s off
  override = out.gasPressed     # driver on the gas while engaged, icons blink

  # distance bars with a hold-off: openpilot can flip the distance setting back for up to 0.1 s right after the button press
  if distance_progress["shown"] is None or hud.leadDistanceBars == distance_progress["shown"]:
    distance_progress.update(shown=hud.leadDistanceBars, pending=None)
  elif hud.leadDistanceBars != distance_progress["pending"]:
    distance_progress.update(pending=hud.leadDistanceBars, frame=frame)
  elif (frame - distance_progress["frame"]) * DT_CTRL >= DISTANCE_HOLD_TIME:
    distance_progress.update(shown=hud.leadDistanceBars, pending=None)
  distance_bars = distance_progress["shown"]

  # lane change ready like openpilot: steering and above the minimum speed
  lca_ready = lat_active and out.vEgo >= LANE_CHANGE_SPEED_MIN

  # lane change in progress: ready, blinker on, and that side's blind spot is clear
  changing_left = lca_ready and left_blinker and not out.leftBlindspot
  changing_right = lca_ready and right_blinker and not out.rightBlindspot
  changing = changing_left or changing_right

  # lane lines from the camera (0x1b5), quality 2/3 means the line is tracked
  left_ok = msg_1b5.get("Info_LftLnQualSta", 0) in (2, 3)
  right_ok = msg_1b5.get("Info_RtLnQualSta", 0) in (2, 3)

  # stock lines stay centered (15) and slide with the lane offset during a lane change, left + right = 30
  # openpilot has no laneChangeFinishing anymore, the whole lane change is laneChangeStarting
  lane_changing = lane_change.state == "laneChangeStarting"

  # lane offset from both tracked lines, or from one line and half a lane when the camera lost the other
  lane_offset = ((msg_1b5["Info_LftLnPosVal"] + msg_1b5["Info_RtLnPosVal"]) / 2 if left_ok and right_ok else
                 msg_1b5["Info_LftLnPosVal"] + LANE_HALF_WIDTH if left_ok else
                 msg_1b5["Info_RtLnPosVal"] - LANE_HALF_WIDTH if right_ok else 0.)

  # crossed the line: the camera relabels the lanes and the offset jumps to the other side, against the lane change direction
  prev_offset = lane_change_progress["offset"]
  jump = lane_offset - prev_offset if prev_offset is not None and (left_ok or right_ok) else 0.
  if not lane_changing:
    lane_change_progress["crossed"] = False
  elif (lane_change.direction == "left" and jump < -LANE_CROSS_JUMP) or (lane_change.direction == "right" and jump > LANE_CROSS_JUMP):
    lane_change_progress["crossed"] = True
  lane_change_progress["offset"] = lane_offset if left_ok or right_ok else None
  crossed = lane_change_progress["crossed"]

  # no usable line: stay centered, openpilot still changes lanes
  left_position = min(max(round(15 - LANE_POSITION_GAIN * lane_offset), 0), 30) if lane_changing and (left_ok or right_ok) else 15

  # lane change assist icon, stock sends the same value to both sides
  lca_icon = (0 if not (main_cruise_enabled or lat_active) else  # hidden: cruise main off and not steering
              1 if not lca_ready else                            # gray: standby, not steering or below 20 mph
              (2 if blink else 0) if changing else               # green blinking: changing lanes
              2)                                                 # green: ready

  msg_161.update({
    # DBC mismatch (cosmetic): the max is 3 but 4 green is used
    # current:   SG_ LKA_ICON : 30|3@1+ (1,0) [0|3] "" XXX
    # corrected: SG_ LKA_ICON : 30|3@1+ (1,0) [0|7] "" XXX
    "LKA_ICON": ((4 if blink_fast else 0) if hud.leftLaneDepart or hud.rightLaneDepart else  # green fast blink: lane departure warning
                 (0, 3, 4, 3)[lkas_icon]),                                                   # 3 gray: available or disengaging, 4 green: steering

    "LFA_ICON": (0, 1, 2, 3 if blink_fast else 0)[lfa_icon],  # 0 hidden, 1 gray standby, 2 green steering, 3/0 white fast blink on disengage

    "HDA_ICON": (0 if not main_cruise_enabled else     # hidden: cruise main off
                 1 if not enabled else                 # gray: SCC canceled
                 (3 if blink else 0) if override else  # white blinking: gas override
                 2),                                   # green: SCC on, steering paused keeps it green like stock

    "DAW_ICON": 0,  # hidden: no coffee cup, openpilot handles driver monitoring

    "CENTERLINE": 1 if lat_active else 0,  # green center path while steering, hidden otherwise

    "LANELINE_LEFT": (4 if hud.leftLaneDepart else                   # orange: departing over this line, openpilot only warns while not steering
                      0 if not lat_active else                       # gray: not steering
                      (6 if blink else 2) if changing else           # green/white blinking: changing lanes, both lines blink
                      6 if lca_ready and not out.leftBlindspot else  # green: lane change possible to this side
                      2),                                            # white: lane change not possible to this side

    "LANELINE_RIGHT": (4 if hud.rightLaneDepart else                   # orange: departing over this line, openpilot only warns while not steering
                       0 if not lat_active else                        # gray: not steering
                       (6 if blink else 2) if changing else            # green/white blinking: changing lanes, both lines blink
                       6 if lca_ready and not out.rightBlindspot else  # green: lane change possible to this side
                       2),                                             # white: lane change not possible to this side

    # DBC mismatch (cosmetic): stock sends 0-30 on both, the max is 15 and 3 instead of the full 6 bits
    # current:   SG_ LANELINE_LEFT_POSITION : 84|6@1+ (1,0) [0|15] "" XXX
    # corrected: SG_ LANELINE_LEFT_POSITION : 84|6@1+ (1,0) [0|63] "" XXX
    # current:   SG_ LANELINE_RIGHT_POSITION : 94|6@1+ (1,0) [0|3] "" XXX
    # corrected: SG_ LANELINE_RIGHT_POSITION : 94|6@1+ (1,0) [0|63] "" XXX
    "LANELINE_LEFT_POSITION": left_position,  # 15 centered, slides toward 0 moving left and 30 moving right

    "LANELINE_RIGHT_POSITION": 30 - left_position,  # mirror of the left position

    "LCA_LEFT_ICON": lca_icon,  # lane change assist icon, same as right

    "LCA_RIGHT_ICON": lca_icon,  # lane change assist icon, same as left

    # DBC mismatch (cosmetic): the value table says 1 is visible, stock sends 2
    # current:   VAL_ 353 LCA_LEFT_ARROW 0 "HIDDEN" 1 "VISIBLE";
    # corrected: VAL_ 353 LCA_LEFT_ARROW 0 "HIDDEN" 2 "VISIBLE";
    # current:   VAL_ 353 LCA_RIGHT_ARROW 0 "HIDDEN" 1 "VISIBLE";
    # corrected: VAL_ 353 LCA_RIGHT_ARROW 0 "HIDDEN" 2 "VISIBLE";
    "LCA_LEFT_ARROW": 2 if changing_left else 0,  # left arrow only while changing lanes left

    "LCA_RIGHT_ARROW": 2 if changing_right else 0,  # right arrow only while changing lanes right

    "LANE_LEFT": 1 if lane_changing and lane_change.direction == "left" and not crossed else 0,  # green target lane while moving left

    "LANE_RIGHT": 1 if lane_changing and lane_change.direction == "right" and not crossed else 0,  # green target lane while moving right

    "LANE_HIGHLIGHT": 1 if lane_changing and crossed else 0,  # green new lane after crossing the line, until the lane change ends

    "LANE_HIGHLIGHT_DISTANCE": 150 if lane_changing and crossed else 0,  # m, stock always draws the highlight 150 m long
  })

  if hud.leftLaneDepart or hud.rightLaneDepart:
    msg_162["VIBRATE"] = 1  # steering wheel vibration on lane departure

  if openpilot_longitudinal_control:
    # 1/3 adjusting to speed limit, 2/4 set speed changed
    if msg_161["ALERTS_3"] in (1, 2, 3, 4):
      msg_161["ALERTS_3"] = 0

    # DBC mismatch (cosmetic): the max is 15 but stock sends up to 28, and 11-14 (driving style distance 1-4) have no labels
    # current:   SG_ ALERTS_3 : 139|5@1+ (1,0) [0|15] "" XXX
    # corrected: SG_ ALERTS_3 : 139|5@1+ (1,0) [0|31] "" XXX
    # missing:   11 "DISTANCE_1_DRIVING_STYLE" 12 "DISTANCE_2_DRIVING_STYLE" 13 "DISTANCE_3_DRIVING_STYLE" 14 "DISTANCE_4_DRIVING_STYLE"
    # the camera still pops up the distance on the distance button (7-10 distance 1-4, 11-14 with driving style)
    # keep its timing but show the openpilot distance, 7-9 distance 1-3
    if 7 <= msg_161["ALERTS_3"] <= 14:
      msg_161["ALERTS_3"] = 6 + distance_bars

    # smart cruise conditions not met
    if msg_161["ALERTS_5"] == 4:
      msg_161["ALERTS_5"] = 0

    # single chime
    if msg_161["SOUNDS_3"] == 5:
      msg_161["SOUNDS_3"] = 0

    scc_on = main_cruise_enabled and enabled
    scc_states = {scc.visionState, scc.mapState}

    # set speed in cluster units, above the cluster max (145 km/h, 90 mph) shows the minimum
    cluster_set_speed = round(out.vCruiseCluster * (1 if is_metric else CV.KPH_TO_MPH))
    set_speed = (40 if is_metric else 25) if cluster_set_speed > (145 if is_metric else 90) else cluster_set_speed

    # speed limit in cluster units, compared to the set speed like Speed Limit Assist does
    sla_limit = round(sl.speedLimitFinalLast * (CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH))
    sla_up = sla_limit >= cluster_set_speed

    # after confirming, SLA writes the new set speed itself and briefly reports inactive, keep it confirmed while the set speed sits on the limit
    sla_asking = sl.assistState in ("preActive", "pending")
    sla_confirmed = sl.assistState in ("active", "adapting") or (sl.assistState == "inactive" and sla_limit == cluster_set_speed)
    if sla_asking:
      sla_progress.update(arrow=1 if sla_up else 2, frame=frame)  # white up/down: waiting to confirm
    elif sla_confirmed and sla_progress["arrow"] in (1, 2):
      sla_progress.update(arrow=sla_progress["arrow"] + 2, frame=frame)  # same arrow turns green on confirm
    elif not sla_confirmed:
      sla_progress["arrow"] = 0  # hidden: speed limit assist off, or set speed changed by hand
    sla_arrow = sla_progress["arrow"]

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

      "DISTANCE": distance_bars if main_cruise_enabled else 0,  # distance bars 1-3, hidden when cruise main off

      # gap graphic: blue is normal distance (popups 7-10), white is SCC machine learning "Based on Driving Style" (popups 11-14)
      # openpilot has no driving style mode, so always blue
      "DISTANCE_SPACING": (0 if not scc_on else                  # hidden: SCC off or canceled
                           (1 if blink else 0) if override else  # blue blinking: gas override
                           1),                                   # blue: SCC on

      "DISTANCE_LEAD": (0 if not main_cruise_enabled or not hud.leadVisible else  # hidden: cruise main off or no lead
                        2 if enabled and lead_following else                      # white: SCC is following this lead
                        1),                                                       # gray: lead seen, cruising at set speed or SCC canceled

      "DISTANCE_CAR": 0 if not main_cruise_enabled else 2 if enabled else 1,  # own car in gap graphic: white SCC on, gray canceled

      "SLA_ICON": (sla_arrow if sla_arrow in (1, 2) or (frame - sla_progress["frame"]) * DT_CTRL < SLA_CONFIRMED_SHOW_TIME else  # green hides after a moment
                   0),

      # DBC mismatch (cosmetic): the max is 3 but 4 white is used
      # current:   SG_ NAV_ICON : 220|4@1+ (1,0) [0|3] "" XXX
      # corrected: SG_ NAV_ICON : 220|4@1+ (1,0) [0|15] "" XXX
      "NAV_ICON": ((2 if blink else 0) if {"entering", "turning"} & scc_states else  # green blinking: slowing for a curve
                   (4 if blink else 0) if "overriding" in scc_states else            # white blinking: gas override
                   2 if {"enabled", "leaving"} & scc_states else                     # green: ready
                   0),                                                               # hidden: vision and map SCC off

      "TARGET": 0,  # hidden: target gap marker, openpilot doesn't hold a fixed gap like stock SCC
    })

    msg_162["LEAD"] = (2 if enabled and lead_following else 1) if main_cruise_enabled and hud.leadVisible else 0  # lead box: white followed, gray seen
    msg_162["LEAD_DISTANCE"] = lead_distance                                                                      # lead distance in m

  # stock long: show ICBM set speed changes, otherwise keep the stock icon (vTarget is in cluster units)
  elif icbm.state in ("preActive", "increasing", "decreasing"):
    msg_161["SLA_ICON"] = (3 if icbm.state == "increasing" else  # green up: ICBM pressing +
                           4 if icbm.state == "decreasing" else  # green down: ICBM pressing -
                           # white up/down: ICBM about to start, arrow points from set speed toward the target
                           1 if icbm.vTarget >= round(out.vCruiseCluster * (1 if is_metric else CV.KPH_TO_MPH)) else 2)

  return [packer.make_can_msg(msg, CAN.ECAN, data) for msg, data in [("CCNC_0x161", msg_161), ("CCNC_0x162", msg_162)]]
