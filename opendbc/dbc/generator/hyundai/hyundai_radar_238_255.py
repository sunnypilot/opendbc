#!/usr/bin/env python3


def _comment(*parts: str) -> str:
  return " ".join(parts)


# Partial decode of the front radar object list seen on bus 1 of the 2021-23 Hyundai Elantra (CN7, CAN, radar SCC).
# Radar FW: "CN7_ SCC F-CUP 1.00 1.01 99110-AA000". Validated on routes 4db9a4d26f328177/000001f9--145655d2f1 (segments 12-13)
# and 4db9a4d26f328177/000001fe--762db300bd (segments 23-24): city driving up to 64 km/h, objects out to about 118 m.
RADAR_238_255_MESSAGE_COMMENT = _comment(
  "CN7 front radar object list on bus 1: ten object slots, three 8-byte messages per slot, approximately 33 Hz.",
  "Slot n uses 0x238+3n (position and speed), 0x239+3n (angles, not decoded) and 0x23A+3n (age and heading).",
  "An empty slot is all zero apart from the shared 2-bit counter. Objects move between slots; slots are not sorted",
  "by distance, so use OBJECT_ID as the track ID.",
  "The stream keeps running on bus 1 while openpilot longitudinal holds the radar quiet on bus 0 via 0x7D0.",
  "Speed is reported over ground: relative speed = ABS_SPEED - ego speed. Ego speed is not on bus 1.",
)

SIGNAL_COMMENTS_1 = (
  ("LONG_DIST", _comment(
    "Longitudinal distance from the radar. Slot 0 matched radarState.leadOne.dRel within 1 m in 81% of frames",
    "(median 0.3 m under 40 m) and lands on the lead vehicle when projected onto qcamera.",
  )),
  ("CLASS", "Object type: 1 truck or large vehicle, 2 car, 5 pedestrian (checked on video); 3 is rare and unconfirmed."),
  ("LAT_DIST", _comment(
    "Lateral position, left positive (openpilot yRel convention); raw value is 2048 at center, raw increases to the right.",
    "Against the vision lead's yRel under 40 m: r = 0.98 / 0.93 / 0.61 on three segments, median difference 0.07-0.14 m. Side confirmed on video.",
  )),
  ("ABS_SPEED", _comment(
    "Longitudinal object speed over ground (0 for stationary objects). REL_SPEED = ABS_SPEED - vEgo:",
    "matches the slope of LONG_DIST with R^2 = 0.94 over all slots in three segments (median error 0.23 m/s, fitted vEgo coefficient -0.99)",
    "and the vision lead's vLead (median difference 0.0-0.3 m/s).",
  )),
  ("COUNTER", "2-bit counter, identical in all three messages of a slot and in every slot for a given cycle; increments by 1 per cycle."),
  ("LAT_SPEED", _comment(
    "Lateral object speed, left positive; raw 500 for stationary objects.",
    "Sign and scale follow the slope of LAT_DIST; noisier than ABS_SPEED.",
  )),
  ("OBJECT_ID", _comment(
    "Track ID, 1-63. Stays with the object when it moves to another slot and is repeated in the other two messages of the slot;",
    "IDs are reused after a track ends.",
  )),
)

SIGNAL_COMMENTS_2 = (
  ("COUNTER", "Same 2-bit counter as the first message of the slot."),
  ("OBJECT_ID", "Same track ID as the first message of the slot."),
)

SIGNAL_COMMENTS_3 = (
  ("COUNTER", "Same 2-bit counter as the first message of the slot."),
  ("AGE", "Track age in radar cycles: starts low when a track appears and increments by 1 per cycle (about 32.5 per second); seen up to 2047."),
  ("OBJECT_ID", "Same track ID as the first message of the slot."),
  ("HEADING", _comment(
    "Object heading relative to the ego vehicle, left (counter-clockwise) positive: 0 = same direction, +-3.14 = facing the ego",
    "vehicle. Raw range 0-628. For moving objects it matches atan2(LAT_SPEED, ABS_SPEED) with a median difference of 1.1 deg",
    "(92% within 10 deg); oncoming and parked-facing cars read about -173 deg, crossing traffic about +89 deg, pedestrians 0.",
  )),
)


def generate():
  parts = []
  parts.append("""
VERSION ""


NS_ :
    NS_DESC_
    CM_
    BA_DEF_
    BA_
    VAL_
    CAT_DEF_
    CAT_
    FILTER
    BA_DEF_DEF_
    EV_DATA_
    ENVVAR_DATA_
    SGTYPE_
    SGTYPE_VAL_
    BA_DEF_SGTYPE_
    BA_SGTYPE_
    SIG_TYPE_REF_
    VAL_TABLE_
    SIG_GROUP_
    SIG_VALTYPE_
    SIGTYPE_VALTYPE_
    BO_TX_BU_
    BA_DEF_REL_
    BA_REL_
    BA_DEF_DEF_REL_
    BU_SG_REL_
    BU_EV_REL_
    BU_BO_REL_
    SG_MUL_VAL_

BS_:

BU_: FR_RDR ADAS_DRV
    """)

  for slot in range(10):
    a = 0x238 + 3 * slot
    parts.append(f"""
BO_ {a} RADAR_TRACK_{a:x}: 8 FR_RDR
 SG_ LONG_DIST : 7|12@0+ (0.1,0) [0|409.5] "m" ADAS_DRV
 SG_ CLASS : 10|3@0+ (1,0) [0|7] "" ADAS_DRV
 SG_ LAT_DIST : 23|12@0+ (-0.05,102.4) [-102.35|102.4] "m" ADAS_DRV
 SG_ ABS_SPEED : 39|12@0+ (0.05,-100) [-100|104.75] "m/s" ADAS_DRV
 SG_ COUNTER : 43|2@0+ (1,0) [0|3] "" ADAS_DRV
 SG_ LAT_SPEED : 41|10@0+ (-0.05,25) [-26.15|25] "m/s" ADAS_DRV
 SG_ OBJECT_ID : 62|6@0+ (1,0) [0|63] "" ADAS_DRV

BO_ {a + 1} RADAR_TRACK_{a + 1:x}: 8 FR_RDR
 SG_ COUNTER : 7|2@0+ (1,0) [0|3] "" ADAS_DRV
 SG_ OBJECT_ID : 61|6@0+ (1,0) [0|63] "" ADAS_DRV

BO_ {a + 2} RADAR_TRACK_{a + 2:x}: 8 FR_RDR
 SG_ COUNTER : 7|2@0+ (1,0) [0|3] "" ADAS_DRV
 SG_ AGE : 39|11@0+ (1,0) [0|2047] "" ADAS_DRV
 SG_ OBJECT_ID : 43|6@0+ (1,0) [0|63] "" ADAS_DRV
 SG_ HEADING : 51|10@0+ (0.01,-3.14) [-3.14|3.14] "rad" ADAS_DRV
    """)

  for slot in range(10):
    a = 0x238 + 3 * slot
    for addr in (a, a + 1, a + 2):
      parts.append(f'CM_ BO_ {addr} "{RADAR_238_255_MESSAGE_COMMENT}";\n')
    for addr, comments in ((a, SIGNAL_COMMENTS_1), (a + 1, SIGNAL_COMMENTS_2), (a + 2, SIGNAL_COMMENTS_3)):
      for signal, comment in comments:
        parts.append(f'CM_ SG_ {addr} {signal} "{comment}";\n')

  for slot in range(10):
    a = 0x238 + 3 * slot
    parts.append(f'VAL_ {a} CLASS 1 "TRUCK" 2 "CAR" 5 "PEDESTRIAN" ;\n')

  return {"hyundai_radar_238_255.dbc": "".join(parts)}
