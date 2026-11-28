#!/usr/bin/env python3


MESSAGE_COMMENT = " ".join((
  "Optional compact auxiliary scan list accompanying the 3A5-3C4 tracked-object range on EV9 and Ioniq 9.",
  "Five messages provide 35 physical record positions; the last three records of 0x3D4 are zero, leaving 32 usable",
  "detections. Record zero starts at bit 24, bit 56 is an observed-zero alignment byte, and records one through six",
  "start at bit 64. Unused records use raw sentinel 0xC8782EE0 or zero. The 12/4/8/8-bit boundaries and a range-like",
  "low-12-bit distribution are observed, but synchronized 3A5 matching falsified the former radial-speed and azimuth",
  "interpretations. None of these fields is a calibrated physical measurement. The records are not independent",
  "tracked objects and must not be published as RadarData tracks.",
))


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
    SIG_GROUP_

BS_:

BU_: FRONT_RADAR ADAS_DRV
"""]

  starts = (24, 64, 96, 128, 160, 192, 224)
  for address in range(0x3D0, 0x3D5):
    parts.append(f"\nBO_ {address} RADAR_DETECTIONS_{address:x}: 32 FRONT_RADAR\n")
    parts.append(' SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" ADAS_DRV\n')
    parts.append(' SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" ADAS_DRV\n')
    parts.append(' SG_ ALIGNMENT : 56|8@1+ (1,0) [0|255] "" ADAS_DRV\n')
    for record, start in enumerate(starts):
      parts.append(f' SG_ DETECTION_{record}_DISTANCE_CANDIDATE : {start}|12@1+ (0.05,0) [0|204.75] "m" ADAS_DRV\n')
      parts.append(f' SG_ DETECTION_{record}_FLAGS_UNKNOWN : {start + 12}|4@1+ (1,0) [0|15] "" ADAS_DRV\n')
      parts.append(f' SG_ DETECTION_{record}_UNKNOWN_BYTE_16 : {start + 16}|8@1+ (1,0) [0|255] "" ADAS_DRV\n')
      parts.append(f' SG_ DETECTION_{record}_UNKNOWN_BYTE_24 : {start + 24}|8@1+ (1,0) [0|255] "" ADAS_DRV\n')
    parts.append(f'CM_ BO_ {address} "{MESSAGE_COMMENT}";\n')

  return {"hyundai_radar_detections_3d0_3d4.dbc": "".join(parts)}
