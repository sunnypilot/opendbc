#!/usr/bin/env python3


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

  for a in range(0x3A5, 0x3A5 + 32):
    parts.append(f"""
BO_ {a} RADAR_TRACK_{a:x}: 24 RADAR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ STATE_ALT : 25|2@0+ (1,0) [0|3] "" XXX
 SG_ MOTION_STATE : 28|2@0+ (1,0) [0|3] "" XXX
 SG_ TRACK_COUNTER : 31|2@0+ (1,0) [0|3] "" XXX
 SG_ NEW_SIGNAL_2 : 38|7@0- (1,0) [-64|63] "" XXX
 SG_ AGE : 47|8@0+ (1,0) [0|255] "" XXX
 SG_ COAST_AGE : 51|4@0+ (1,0) [0|15] "" XXX
 SG_ STATE : 54|3@0+ (1,0) [0|7] "" XXX
 SG_ NEW_SIGNAL_8 : 62|7@0- (1,0) [-64|63] "" XXX
 SG_ LONG_DIST : 63|12@1+ (0.05,0) [0|204.75] "m" XXX
 SG_ LAT_DIST : 76|12@1- (0.05,0) [-102.4|102.35] "m" XXX
 SG_ REL_SPEED : 88|14@1- (0.01,0) [-81.92|81.91] "m/s" XXX
 SG_ NEW_SIGNAL_4 : 103|2@0+ (1,0) [0|3] "" XXX
 SG_ REL_LAT_SPEED : 104|13@1- (0.01,0) [-40.96|40.95] "m/s" XXX
 SG_ REL_ACCEL : 118|10@1- (0.02,0) [-10.24|10.22] "m/s^2" XXX
 SG_ NEW_SIGNAL_5 : 134|5@0+ (1,0) [0|31] "" XXX

VAL_ {a} MOTION_STATE 0 "UNKNOWN" 1 "STATIONARY" 2 "MOVING" ;
    """)

  return {"hyundai_radar_3a5_3c4.dbc": "".join(parts)}
