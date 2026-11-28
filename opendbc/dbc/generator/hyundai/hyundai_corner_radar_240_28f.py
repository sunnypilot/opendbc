#!/usr/bin/env python3


DBC_HEADER = """
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

BU_: CORNER_RADAR_A CORNER_RADAR_B ADAS_DRV
"""

CHANNEL_SIDES = {
  "A": "right",
  "B": "left",
}


def _comment(*parts: str) -> str:
  return " ".join(parts)


DETECTION_COMMENT = _comment(
  "Seven packed 32-bit compact scan records. Record zero starts at bit 24, bit 56 is a frame-status/alignment byte,",
  "and records one through six start at bit 64. DISTANCE_CANDIDATE has a range-like 0.05-unit distribution and a",
  "400 m endpoint, but physical distance is not confirmed. DETECTION_PROPERTY and DETECTION_AUX are active unknowns.",
  "Same-cycle range matching is no better than opposite-channel",
  "or time-shifted controls, and linear bin-angle searches also fail those controls. Each position has a characteristic",
  "occupancy profile; treat this as a candidate fixed scan/bin array rather than freely assigned tracked detections.",
  "Unused records commonly contain raw 0x010D1F40, with variants such as 0x018D1F40."
)


def _channel_comment(channel: str) -> str:
  side = CHANNEL_SIDES[channel]
  return _comment(
    f"Candidate HDA2 {side}-front corner-radar channel. The physical side is strongly inferred from a synchronized",
    "Palisade parking-garage sweep, but still awaits a controlled sensor-occlusion test. Each channel contains one",
    "status message, fifteen 24-byte object slots, and fifty-six compact scan records in eight 32-byte messages at",
    "approximately 20 Hz. The HKG CAN-FD checksum validated on Ioniq 5, Palisade 2023, and EV6 routes. Candidate",
    "fields must not be used for control or filtering."
  )


def _status_message(address: int, size: int, channel: str) -> str:
  transmitter = f"CORNER_RADAR_{channel}"
  parts = [f"\nBO_ {address} CORNER_{channel}_STATUS_{address:x}: {size} {transmitter}\n"]
  parts.append(' SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" ADAS_DRV\n')
  parts.append(' SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" ADAS_DRV\n')
  first_unknown_byte = 3
  if channel == "B":
    parts.append(' SG_ CHANNEL_A_OBJECT_COUNT : 24|8@1+ (1,0) [0|255] "" ADAS_DRV\n')
    first_unknown_byte = 4
  for byte in range(first_unknown_byte, size):
    parts.append(f' SG_ UNKNOWN_BYTE_{byte} : {byte * 8}|8@1+ (1,0) [0|255] "" ADAS_DRV\n')
  status_comment = _channel_comment(channel)
  if channel == "B":
    status_comment = _comment(
      status_comment,
      "CHANNEL_A_OBJECT_COUNT is the pre-truncation count for the preceding 15-slot A object list. Across 5,999",
      "synchronized cycles on five Ioniq 5, Palisade, and EV6 samples, the number of non-sentinel A slots equaled",
      "min(count, 15) exactly. EV6 reached 22, proving the byte includes internal candidates not exported in the 15 slots.",
    )
  parts.append(f'CM_ BO_ {address} "{status_comment}";\n')
  return "".join(parts)


def _object_message(address: int, channel: str, slot: int) -> str:
  transmitter = f"CORNER_RADAR_{channel}"
  parts = [f"\nBO_ {address} CORNER_{channel}_OBJECT_{slot:02d}: 24 {transmitter}\n"]
  parts.append(' SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" ADAS_DRV\n')
  parts.append(' SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" ADAS_DRV\n')
  parts.append(' SG_ UNKNOWN_CATEGORY : 25|2@0+ (1,0) [0|3] "" ADAS_DRV\n')
  parts.append(' SG_ UNKNOWN_BITS_26_55 : 26|30@1+ (1,0) [0|1073741823] "" ADAS_DRV\n')
  parts.append(' SG_ RCS : 62|7@0- (1,0) [-64|63] "" ADAS_DRV\n')
  parts.append(' SG_ LONG_DIST : 63|13@1+ (0.05,0) [0|409.55] "m" ADAS_DRV\n')
  parts.append(' SG_ LAT_DIST : 76|11@1- (0.05,0) [-51.2|51.15] "m" ADAS_DRV\n')
  parts.append(' SG_ UNKNOWN_BIT_87 : 87|1@1+ (1,0) [0|1] "" ADAS_DRV\n')
  parts.append(' SG_ REL_SPEED : 88|10@1- (0.2,0) [-102.4|102.2] "m/s" ADAS_DRV\n')
  parts.append(' SG_ REL_LAT_SPEED : 98|10@1- (0.05,0) [-25.6|25.55] "m/s" ADAS_DRV\n')
  parts.append(' SG_ UNKNOWN_BITS_108_117 : 108|10@1+ (1,0) [0|1023] "" ADAS_DRV\n')
  parts.append(' SG_ UNKNOWN_BITS_118_127 : 118|10@1+ (1,0) [0|1023] "" ADAS_DRV\n')
  parts.append(' SG_ UNKNOWN_BITS_128_191 : 128|64@1+ (1,0) [0|18446744073709551615] "" ADAS_DRV\n')
  object_comment = _comment(
    f"{_channel_comment(channel)} LONG_DIST, LAT_DIST, REL_SPEED, and REL_LAT_SPEED are route-validated vehicle-frame",
    "kinematics. UNKNOWN_CATEGORY is stable during continuous same-slot tracks and changes mainly when the slot acquires",
    "a spatially different target; all values 0-3 occur on active objects, so it is not lifecycle or validity. Empty slots",
    "use the 204.7 m LONG_DIST sentinel. RCS shares the front-radar field position and distribution.",
    "REL_SPEED is signed 10-bit. Four held-out far targets exercised bit 97 independently of bit 96 and their",
    "distance derivatives matched the extended negative-speed decode, disproving the former 9-bit boundary.",
    "The former acceleration candidate at bits 108-117 failed derivative validation and is preserved raw.",
  )
  parts.append(f'CM_ BO_ {address} "{object_comment}";\n')
  return "".join(parts)


def _detection_message(address: int, channel: str, block: int) -> str:
  transmitter = f"CORNER_RADAR_{channel}"
  starts = (24, 64, 96, 128, 160, 192, 224)
  parts = [f"\nBO_ {address} CORNER_{channel}_DETECTIONS_{block}: 32 {transmitter}\n"]
  parts.append(' SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" ADAS_DRV\n')
  parts.append(' SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" ADAS_DRV\n')
  parts.append(' SG_ FRAME_STATUS : 56|8@1+ (1,0) [0|255] "" ADAS_DRV\n')
  for record, start in enumerate(starts):
    parts.append(f' SG_ DETECTION_{record}_DISTANCE_CANDIDATE : {start}|13@1+ (0.05,0) [0|409.55] "m" ADAS_DRV\n')
    parts.append(f' SG_ DETECTION_{record}_RESERVED : {start + 13}|3@1+ (1,0) [0|7] "" ADAS_DRV\n')
    parts.append(f' SG_ DETECTION_{record}_PROPERTY : {start + 16}|7@1+ (1,0) [0|127] "" ADAS_DRV\n')
    parts.append(f' SG_ DETECTION_{record}_AUX : {start + 23}|9@1+ (1,0) [0|511] "" ADAS_DRV\n')
  parts.append(f'CM_ BO_ {address} "{_channel_comment(channel)} {DETECTION_COMMENT}";\n')
  return "".join(parts)


def generate():
  parts = [DBC_HEADER]
  parts.append(_status_message(0x240, 16, "A"))
  for slot, address in enumerate(range(0x241, 0x250), start=1):
    parts.append(_object_message(address, "A", slot))
  for block, address in enumerate(range(0x270, 0x278)):
    parts.append(_detection_message(address, "A", block))

  parts.append(_status_message(0x278, 8, "B"))
  for slot, address in enumerate(range(0x279, 0x288), start=1):
    parts.append(_object_message(address, "B", slot))
  for block, address in enumerate(range(0x288, 0x290)):
    parts.append(_detection_message(address, "B", block))

  return {"hyundai_corner_radar_240_28f.dbc": "".join(parts)}
