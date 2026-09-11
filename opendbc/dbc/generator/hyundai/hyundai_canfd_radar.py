#!/usr/bin/env python3


def generate():
  parts = ["""
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

BS_:

BU_: XXX
  """]

  # This CAN-FD layout carries two independent tracked objects per frame.
  # Signal positions and scaling are validated against IONIQ 5 driving logs.
  for addr in range(0x210, 0x220):
    parts.append(f"""
BO_ {addr} RADAR_TRACK_{addr:x}: 32 RADAR
 SG_ VALID_CNT1 : 47|8@0+ (1,0) [0|255] "" XXX
 SG_ LONG_DIST1 : 64|12@1+ (0.05,0) [0|204.75] "m" XXX
 SG_ LAT_DIST1 : 76|12@1- (0.05,0) [-102.4|102.35] "m" XXX
 SG_ REL_SPEED1 : 88|14@1- (0.01,0) [-81.92|81.91] "m/s" XXX
 SG_ LAT_SPEED1 : 104|13@1- (0.01,0) [-40.96|40.95] "m/s" XXX
 SG_ REL_ACCEL1 : 118|10@1- (0.05,0) [-25.6|25.55] "m/s2" XXX
 SG_ VALID_CNT2 : 175|8@0+ (1,0) [0|255] "" XXX
 SG_ LONG_DIST2 : 192|12@1+ (0.05,0) [0|204.75] "m" XXX
 SG_ LAT_DIST2 : 204|12@1- (0.05,0) [-102.4|102.35] "m" XXX
 SG_ REL_SPEED2 : 216|14@1- (0.01,0) [-81.92|81.91] "m/s" XXX
 SG_ LAT_SPEED2 : 232|13@1- (0.01,0) [-40.96|40.95] "m/s" XXX
 SG_ REL_ACCEL2 : 246|10@1- (0.05,0) [-25.6|25.55] "m/s2" XXX
    """)

  return {"hyundai_canfd_radar.dbc": "".join(parts)}
