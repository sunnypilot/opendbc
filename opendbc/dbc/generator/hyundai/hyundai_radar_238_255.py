#!/usr/bin/env python3


def _comment(*parts: str) -> str:
  return " ".join(parts)


# Partial decode of the front radar object list seen on bus 1 of the 2021-23 Hyundai Elantra (CN7, CAN, radar SCC).
# Radar FW: "CN7_ SCC F-CUP 1.00 1.01 99110-AA000". Validated on route 4db9a4d26f328177/000001f9--145655d2f1 (segments 12-13).
RADAR_238_255_MESSAGE_COMMENT = _comment(
  "CN7 front radar object list on bus 1: ten object slots, three 8-byte messages per slot, approximately 33 Hz.",
  "Slot n uses 0x238+3n (position), 0x239+3n and 0x23A+3n (not yet decoded).",
  "An empty slot is all zero apart from the shared 2-bit counter.",
  "The stream keeps running on bus 1 while openpilot longitudinal holds the radar quiet on bus 0 via 0x7D0.",
  "Partial decode: relative speed has not been identified yet.",
)

POSITION_SIGNAL_COMMENTS = (
  ("LONG_DIST", _comment(
    "Longitudinal distance from the radar. Slot 0 matched radarState.leadOne.dRel within 1 m in 81% of frames",
    "(median 0.3 m under 40 m) and lands on the lead vehicle when projected onto qcamera.",
  )),
  ("LAT_DIST", _comment(
    "Lateral position, left positive (openpilot yRel convention); raw value is 2048 at center, raw increases to the right.",
    "Fit against the vision lead: r = -0.98 under 40 m. Side confirmed on video; scale is approximate.",
  )),
  ("COUNTER", "2-bit counter, identical in all three messages of a slot and in every slot for a given cycle; increments by 1 per cycle."),
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
 SG_ LAT_DIST : 23|12@0+ (-0.05,102.4) [-102.35|102.4] "m" ADAS_DRV
 SG_ COUNTER : 43|2@0+ (1,0) [0|3] "" ADAS_DRV

BO_ {a + 1} RADAR_TRACK_{a + 1:x}: 8 FR_RDR
 SG_ COUNTER : 7|2@0+ (1,0) [0|3] "" ADAS_DRV

BO_ {a + 2} RADAR_TRACK_{a + 2:x}: 8 FR_RDR
 SG_ COUNTER : 7|2@0+ (1,0) [0|3] "" ADAS_DRV
    """)

  for slot in range(10):
    a = 0x238 + 3 * slot
    for addr in (a, a + 1, a + 2):
      parts.append(f'CM_ BO_ {addr} "{RADAR_238_255_MESSAGE_COMMENT}";\n')
    for signal, comment in POSITION_SIGNAL_COMMENTS:
      parts.append(f'CM_ SG_ {a} {signal} "{comment}";\n')

  return {"hyundai_radar_238_255.dbc": "".join(parts)}
