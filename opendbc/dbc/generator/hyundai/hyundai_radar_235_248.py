#!/usr/bin/env python3


def _comment(*parts: str) -> str:
  return " ".join(parts)


CAMERA_235_248_MESSAGE_COMMENT = _comment(
  "Forward-camera object list, not a raw radar-track range.",
  "The core through bit 123 exactly matches Hyundai/Veoneer FR_CMR object messages:",
  "quality, age, motion class, persistent object ID, width, classification, relative position, velocity, and acceleration.",
  "Twenty 32-byte messages provide one object slot each at approximately 33 Hz.",
  "The extension at bits 125-160 remains unknown; the high-resolution azimuth at bit 176 is route-validated.",
  "Some platforms use a checksum variant that is not the standard HKG CAN-FD checksum.",
)

CAMERA_235_248_SIGNAL_COMMENTS = (
  ("CHECKSUM", "16-bit message checksum. HKG CAN-FD validates on some, but not all, observed platforms."),
  ("COUNTER", "8-bit transmit-cycle counter."),
  ("QUALITY", "Unsigned 7-bit camera-object quality level. The related FR_CMR definition uses a nominal 0-100 scale."),
  ("AGE", "Per-object alive age."),
  ("MOTION_STATE", _comment(
    "Camera-object motion class: 0=undefined, 1=standing, 2=parked, 3=stopped, 4=unknown movable,",
    "5=moving, 6=stopped oncoming, 7=unknown oncoming, 8=moving oncoming, 9=crossing bicycle.",
  )),
  ("OBJECT_ID", "Persistent 7-bit camera-object identifier. Zero denotes an unused slot in the observed routes."),
  ("WIDTH", "Estimated object width."),
  ("CLASSIFICATION", "0=unknown, 1=truck, 2=car, 3=motorcycle, 4=bicycle, 5=pedestrian, 6=undecided."),
  ("LONG_DIST", "Camera-object longitudinal relative position."),
  ("LAT_DIST", "Camera-object lateral relative position."),
  ("REL_SPEED", "Camera-object longitudinal velocity relative to ego."),
  ("REL_LAT_SPEED", "Camera-object lateral velocity relative to ego."),
  ("REL_ACCEL", "Camera-object longitudinal acceleration relative to ego."),
  ("UNKNOWN_1", "Active extension field with unknown semantics; preserve as raw."),
  ("UNKNOWN_2", "Active extension field with unknown semantics; preserve as raw."),
  ("UNKNOWN_3", "Active extension field with unknown semantics; preserve as raw."),
  ("AZIMUTH", "High-resolution object azimuth over a full 360-degree unsigned encoding."),
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

BU_: FR_CMR ADAS_DRV
    """)

  for a in range(0x235, 0x235 + 20):
    parts.append(f"""
BO_ {a} RADAR_TRACK_{a:x}: 32 FR_CMR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" ADAS_DRV
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" ADAS_DRV
 SG_ QUALITY : 24|7@1+ (1,0) [0|127] "" ADAS_DRV
 SG_ AGE : 32|8@1+ (1,0) [0|255] "" ADAS_DRV
 SG_ MOTION_STATE : 40|4@1+ (1,0) [0|9] "" ADAS_DRV
 SG_ OBJECT_ID : 44|7@1+ (1,0) [0|127] "" ADAS_DRV
 SG_ WIDTH : 52|7@1+ (0.05,0) [0|6.35] "m" ADAS_DRV
 SG_ CLASSIFICATION : 60|3@1+ (1,0) [0|6] "" ADAS_DRV
 SG_ LONG_DIST : 64|13@1+ (0.05,0) [0|409.55] "m" ADAS_DRV
 SG_ LAT_DIST : 78|12@1+ (0.05,-102.4) [-102.4|102.35] "m" ADAS_DRV
 SG_ REL_SPEED : 91|12@1+ (0.05,-100) [-100|104.75] "m/s" ADAS_DRV
 SG_ REL_LAT_SPEED : 104|10@1+ (0.05,-25) [-25|26.15] "m/s" ADAS_DRV
 SG_ REL_ACCEL : 115|9@1- (0.05,0) [-12.8|12.75] "m/s^2" ADAS_DRV
 SG_ UNKNOWN_1 : 125|12@1+ (1,0) [0|4095] "" ADAS_DRV
 SG_ UNKNOWN_2 : 138|12@1+ (1,0) [0|4095] "" ADAS_DRV
 SG_ UNKNOWN_3 : 151|10@1+ (1,0) [0|1023] "" ADAS_DRV
 SG_ AZIMUTH : 176|14@1+ (0.02197265625,-180) [-180|179.97802734375] "deg" ADAS_DRV
    """)

  for a in range(0x235, 0x235 + 20):
    parts.append(f'CM_ BO_ {a} "{CAMERA_235_248_MESSAGE_COMMENT}";\n')
    for signal, comment in CAMERA_235_248_SIGNAL_COMMENTS:
      parts.append(f'CM_ SG_ {a} {signal} "{comment}";\n')
    parts.append("".join((
      f'VAL_ {a} MOTION_STATE 0 "Undefined" 1 "Standing" 2 "Parked" 3 "Stopped" ',
      '4 "Unknown movable" 5 "Moving" 6 "Stopped oncoming" 7 "Unknown oncoming" ',
      '8 "Moving oncoming" 9 "Crossing bicycle";\n',
    )))
    parts.append("".join((
      f'VAL_ {a} CLASSIFICATION 0 "Unknown" 1 "Truck" 2 "Car" 3 "Motorcycle" ',
      '4 "Bicycle" 5 "Pedestrian" 6 "Undecided";\n',
    )))

  return {"hyundai_radar_235_248.dbc": "".join(parts)}
