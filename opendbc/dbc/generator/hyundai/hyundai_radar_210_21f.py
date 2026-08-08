#!/usr/bin/env python3


def _comment(*parts: str) -> str:
  return " ".join(parts)


RADAR_210_21F_MESSAGE_COMMENT = _comment(
  "Hyundai/Kia 210-21F radar-track layout: 16 messages with two target slots per 32-byte message (32 tracks total).",
  "Two related dialects share the same position, velocity, age, coast-age, and compressed-state core.",
  "The detailed dialect also populates STATE, RCS, and NEW_SIGNAL_4;",
  "the compact Tucson/Sportage/Santa Cruz dialect leaves those fields zero and uses STATE_ALT.",
  "The dialect is independent of the logged raw bus and is not an HDA-level discriminator:",
  "HDA2-like routes carried the sustained stream on A-CAN mapped to bus 0, while non-HDA2 routes mapped A-CAN to bus 1.",
  "Compact targets populate per-slot NEW_SIGNAL_18/OBJECT_ID extension bytes at bytes 16 and 17;",
  "detailed platforms usually leave them zero. Byte 18 and bits 26, 30-31, 39, 55, and 117 remained zero.",
  "REL_LAT_SPEED has opposite sign conventions between the detailed and compact dialects.",
  "The HKG CAN-FD checksum matched every one of 13,210,543 checked frames.",
)

RADAR_210_21F_SIGNAL_COMMENTS = (
  ("CHECKSUM", "16-bit HKG CAN-FD checksum using the CAN address as the data ID."),
  ("COUNTER", "8-bit transmit-cycle counter. It covers 0-255 and wraps."),
  ("STATE_ALT", "Compressed lifecycle state: 0=empty, 1=tentative, 2=measured, 3=coasted."),
  ("MOTION_STATE", _comment(
    "Ground-frame target-motion class: 0=unknown, 1=stationary, 2=moving,",
    "3=stopped, and 4=oncoming. The stopped label is independently supported by related Hyundai BN7 radar and HMVS4 object DBCs.",
    "Values 3/4 were observed only on the compact dialect;",
    "the detailed dialect has no dedicated oncoming value.",
  )),
  ("TRACK_QUALITY", _comment(
    "Unsigned 7-bit track quality/existence score. Detailed platforms share the same lifecycle-dependent distribution",
    "as the 3A5-3C4 layout; compact platforms use a higher calibration that falls as COAST_AGE increases.",
    "A related Hyundai object layout defines its 7-bit quality level as reliability, validity, or probability evidence;",
    "the raw radar scale is not confirmed to be a percentage.",
  )),
  ("AGE", "Per-track alive age/lifetime count, observed 0-255. It normally increments on an update and may hold."),
  ("COAST_AGE", "Prediction age: normally 0 while measured, then increments from 1 to 15 while coasted."),
  ("STATE", _comment(
    "Detailed lifecycle state: 0=empty, 1/2=tentative acquisition, 3=measured, 4=coasted/predicted,",
    "7=unresolved tentative. Values 5 and 6 were not observed; compact platforms leave this field zero.",
  )),
  ("RCS", _comment(
    "Signed radar cross-section/return-strength value. Its distribution matches the 3A5-3C4 RCS field,",
    "and distance-controlled samples consistently increase from small to passenger to large targets.",
    "The raw unit is not calibrated; compact platforms leave it zero.",
  )),
  ("LONG_DIST", "Longitudinal distance from the ego radar. Empty compact slots may contain the 204.75 m endpoint sentinel."),
  ("LAT_DIST", "Lateral position relative to the ego axis."),
  ("REL_SPEED", "Longitudinal speed relative to ego; negative means the target is moving longitudinally slower than ego."),
  ("NEW_SIGNAL_4", "Unknown category, observed 0-2 on detailed platforms. Compact platforms leave it zero."),
  ("REL_LAT_SPEED", _comment(
    "Target lateral speed relative to the ego axis at 0.01 m/s per bit.",
    "The detailed dialect uses the decoded sign; compact platforms use the opposite sign.",
  )),
  ("REL_ACCEL", "Target longitudinal acceleration relative to ego acceleration at 0.05 m/s^2 per bit."),
  ("NEW_SIGNAL_18", _comment(
    "Unknown compact-dialect per-target status area, observed only as 0/1; its second bit remained zero.",
    "Detailed platforms leave it zero.",
  )),
  ("OBJECT_ID", _comment(
    "Strongly inferred compact-dialect persistent object identifier, observed 0-63.",
    "It held through 99.9% of consecutive same-track updates and changed when a slot acquired a replacement target.",
    "A related Hyundai BN7 radar DBC independently defines ObjectId as a global ID that remains constant while an object is tracked.",
    "Detailed platforms usually leave it zero.",
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

BU_: XXX
    """)

  for a in range(0x210, 0x210 + 16):
    parts.append(f"""
BO_ {a} RADAR_TRACK_{a:x}: 32 RADAR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ 1_STATE_ALT : 25|2@0+ (1,0) [0|3] "" XXX
 SG_ 1_MOTION_STATE : 29|3@0+ (1,0) [0|4] "" XXX
 SG_ 1_TRACK_QUALITY : 38|7@0+ (1,0) [0|127] "" XXX
 SG_ 1_AGE : 47|8@0+ (1,0) [0|255] "" XXX
 SG_ 1_COAST_AGE : 51|4@0+ (1,0) [0|15] "" XXX
 SG_ 1_STATE : 54|3@0+ (1,0) [0|7] "" XXX
 SG_ 1_RCS : 63|8@0- (1,0) [-128|127] "" XXX
 SG_ 1_LONG_DIST : 64|12@1+ (0.05,0) [0|204.75] "m" XXX
 SG_ 1_LAT_DIST : 76|12@1- (0.05,0) [-102.4|102.35] "m" XXX
 SG_ 1_REL_SPEED : 88|14@1- (0.01,0) [-81.92|81.91] "m/s" XXX
 SG_ 1_NEW_SIGNAL_4 : 102|2@1+ (1,0) [0|2] "" XXX
 SG_ 1_REL_LAT_SPEED : 104|13@1- (0.01,0) [-40.96|40.95] "m/s" XXX
 SG_ 1_REL_ACCEL : 118|10@1- (0.05,0) [-25.6|25.55] "m/s^2" XXX
 SG_ 1_NEW_SIGNAL_18 : 129|2@0+ (1,0) [0|1] "" XXX
 SG_ 1_OBJECT_ID : 135|6@0+ (1,0) [0|63] "" XXX
 SG_ 2_NEW_SIGNAL_18 : 137|2@0+ (1,0) [0|1] "" XXX
 SG_ 2_OBJECT_ID : 143|6@0+ (1,0) [0|63] "" XXX
 SG_ 2_STATE_ALT : 153|2@0+ (1,0) [0|3] "" XXX
 SG_ 2_MOTION_STATE : 157|3@0+ (1,0) [0|4] "" XXX
 SG_ 2_TRACK_QUALITY : 166|7@0+ (1,0) [0|127] "" XXX
 SG_ 2_AGE : 175|8@0+ (1,0) [0|255] "" XXX
 SG_ 2_COAST_AGE : 179|4@0+ (1,0) [0|15] "" XXX
 SG_ 2_STATE : 182|3@0+ (1,0) [0|7] "" XXX
 SG_ 2_RCS : 191|8@0- (1,0) [-128|127] "" XXX
 SG_ 2_LONG_DIST : 192|12@1+ (0.05,0) [0|204.75] "m" XXX
 SG_ 2_LAT_DIST : 204|12@1- (0.05,0) [-102.4|102.35] "m" XXX
 SG_ 2_REL_SPEED : 216|14@1- (0.01,0) [-81.92|81.91] "m/s" XXX
 SG_ 2_NEW_SIGNAL_4 : 230|2@1+ (1,0) [0|2] "" XXX
 SG_ 2_REL_LAT_SPEED : 232|13@1- (0.01,0) [-40.96|40.95] "m/s" XXX
 SG_ 2_REL_ACCEL : 246|10@1- (0.05,0) [-25.6|25.55] "m/s^2" XXX

VAL_ {a} 1_STATE_ALT 0 "EMPTY" 1 "TENTATIVE" 2 "MEASURED" 3 "COASTED" ;
VAL_ {a} 2_STATE_ALT 0 "EMPTY" 1 "TENTATIVE" 2 "MEASURED" 3 "COASTED" ;
VAL_ {a} 1_MOTION_STATE 0 "UNKNOWN" 1 "STATIONARY" 2 "MOVING" 3 "STOPPED" 4 "ONCOMING" ;
VAL_ {a} 2_MOTION_STATE 0 "UNKNOWN" 1 "STATIONARY" 2 "MOVING" 3 "STOPPED" 4 "ONCOMING" ;
VAL_ {a} 1_STATE 0 "EMPTY" 1 "TENTATIVE_1" 2 "TENTATIVE_2" 3 "MEASURED" 4 "COASTED" 7 "UNRESOLVED_TENTATIVE" ;
VAL_ {a} 2_STATE 0 "EMPTY" 1 "TENTATIVE_1" 2 "TENTATIVE_2" 3 "MEASURED" 4 "COASTED" 7 "UNRESOLVED_TENTATIVE" ;
    """)

    parts.append(f'CM_ BO_ {a} "{RADAR_210_21F_MESSAGE_COMMENT}";\n')
    for prefix in ("1_", "2_"):
      parts.extend(
        f'CM_ SG_ {a} {prefix}{signal} "{comment}";\n'
        for signal, comment in RADAR_210_21F_SIGNAL_COMMENTS
        if signal not in ("CHECKSUM", "COUNTER")
      )
    parts.extend(
      f'CM_ SG_ {a} {signal} "{comment}";\n'
      for signal, comment in RADAR_210_21F_SIGNAL_COMMENTS
      if signal in ("CHECKSUM", "COUNTER")
    )

  return {"hyundai_radar_210_21f.dbc": "".join(parts)}
