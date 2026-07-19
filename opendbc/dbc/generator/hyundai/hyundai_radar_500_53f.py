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

  parts.append("""
BO_ 1248 RADAR_STATUS_4e0: 8 RADAR
 SG_ GROUP_COUNTER : 6|2@1+ (1,0) [0|3] "" XXX
 SG_ DSP_TIMESTAMP : 5|7@0+ (2,0) [0|254] "ms" XXX
 SG_ COMM_ERROR : 14|1@0+ (1,0) [0|1] "" XXX
 SG_ RADIUS_CURVATURE : 13|14@0- (1,0) [-8192|8191] "m" XXX
 SG_ SCAN_INDEX : 31|16@0+ (1,0) [0|65535] "" XXX
 SG_ YAW_RATE : 47|12@0- (0.0625,0) [-128|127.9375] "deg/s" XXX
 SG_ VEHICLE_SPEED : 50|11@0+ (0.0625,0) [0|127.9375] "m/s" XXX

BO_ 1249 RADAR_STATUS_4e1: 8 RADAR
 SG_ GROUP_COUNTER : 1|2@0+ (1,0) [0|3] "" XXX
 SG_ MAXIMUM_TRACKS_ACK : 7|6@0+ (1,1) [1|64] "" XXX
 SG_ STEERING_ANGLE_ACK : 10|11@0+ (1,0) [0|2047] "deg" XXX
 SG_ RAW_DATA_MODE : 11|1@0+ (1,0) [0|1] "" XXX
 SG_ TRANSCEIVER_OPERATIONAL : 12|1@0+ (1,0) [0|1] "" XXX
 SG_ INTERNAL_ERROR : 13|1@0+ (1,0) [0|1] "" XXX
 SG_ RANGE_PERFORMANCE_ERROR : 14|1@0+ (1,0) [0|1] "" XXX
 SG_ OVERHEAT_ERROR : 15|1@0+ (1,0) [0|1] "" XXX
 SG_ TEMPERATURE : 31|8@0- (1,0) [-128|127] "degC" XXX
 SG_ GROUPING_MODE : 33|2@0+ (1,0) [0|3] "" XXX
 SG_ VEHICLE_SPEED_COMP_FACTOR : 39|6@0- (0.00195,1) [0.9376|1.06045] "" XXX
 SG_ YAW_RATE_BIAS : 47|8@0- (0.125,0) [-16|15.875] "deg/s" XXX
 SG_ DSP_SOFTWARE_VERSION : 55|16@0+ (1,0) [0|65535] "" XXX

BO_ 1250 RADAR_BUILD_TIMESTAMP_4e2: 8 RADAR
 SG_ BUILD_YEAR_BCD : 0|8@1+ (1,0) [0|255] "" XXX
 SG_ BUILD_MONTH_BCD : 8|8@1+ (1,0) [0|255] "" XXX
 SG_ BUILD_DAY_BCD : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ BUILD_HOUR_BCD : 24|8@1+ (1,0) [0|255] "" XXX
 SG_ BUILD_MINUTE_BCD : 32|8@1+ (1,0) [0|255] "" XXX
 SG_ UNKNOWN_BYTE_5 : 40|8@1+ (1,0) [0|255] "" XXX
 SG_ UNKNOWN_BYTE_6 : 48|8@1+ (1,0) [0|255] "" XXX
 SG_ UNKNOWN_BYTE_7 : 56|8@1+ (1,0) [0|255] "" XXX

BO_ 1251 RADAR_STATUS_4e3: 8 RADAR
 SG_ GROUP_COUNTER : 1|2@0+ (1,0) [0|3] "" XXX
 SG_ MEDIUM_LONG_RANGE_MODE : 3|2@0+ (1,0) [0|3] "" XXX
 SG_ PARTIAL_BLOCKAGE : 4|1@0+ (1,0) [0|1] "" XXX
 SG_ SIDELOBE_BLOCKAGE : 5|1@0+ (1,0) [0|1] "" XXX
 SG_ LONG_RANGE_GRATING_LOBE_DETECTED : 6|1@0+ (1,0) [0|1] "" XXX
 SG_ TRUCK_TARGET_DETECTED : 7|1@0+ (1,0) [0|1] "" XXX
 SG_ ACC_MOVING_PATH_ID : 15|8@0+ (1,0) [0|255] "" XXX
 SG_ CMBB_MOVING_PATH_ID : 23|8@0+ (1,0) [0|255] "" XXX
 SG_ CMBB_STATIONARY_PATH_ID : 31|8@0+ (1,0) [0|255] "" XXX
 SG_ FCW_MOVING_PATH_ID : 39|8@0+ (1,0) [0|255] "" XXX
 SG_ FCW_STATIONARY_PATH_ID : 47|8@0+ (1,0) [0|255] "" XXX
 SG_ AUTO_ALIGN_ANGLE : 55|8@0- (0.0625,0) [-8|7.9375] "deg" XXX
 SG_ ACC_STATIONARY_PATH_ID : 63|8@0+ (1,0) [0|255] "" XXX

BO_ 1252 RADAR_STATUS_4e4: 8 RADAR
 SG_ SWITCHED_BATTERY_ADC : 0|8@1+ (1,0) [0|255] "raw" XXX
 SG_ IGNITION_ADC : 8|8@1+ (1,0) [0|255] "raw" XXX
 SG_ TEMPERATURE_1_ADC : 16|8@1+ (1,0) [0|255] "raw" XXX
 SG_ TEMPERATURE_2_ADC : 24|8@1+ (1,0) [0|255] "raw" XXX
 SG_ SUPPLY_5VA_ADC : 32|8@1+ (1,0) [0|255] "raw" XXX
 SG_ SUPPLY_5VDX_ADC : 40|8@1+ (1,0) [0|255] "raw" XXX
 SG_ SUPPLY_3V3_ADC : 48|8@1+ (1,0) [0|255] "raw" XXX
 SG_ SUPPLY_10V_ADC : 56|8@1+ (1,0) [0|255] "raw" XXX

BO_ 1253 RADAR_STATUS_4e5: 8 RADAR
 SG_ SUPPLY_1V8_ADC : 0|8@1+ (1,0) [0|255] "raw" XXX
 SG_ SUPPLY_NEGATIVE_5V_ADC : 8|8@1+ (1,0) [0|255] "raw" XXX
 SG_ WAVE_DIFFERENCE_ADC : 16|8@1+ (1,0) [0|255] "raw" XXX
 SG_ SYSTEM_POWER_MODE : 24|3@1+ (1,0) [0|7] "" XXX
 SG_ VERTICAL_ALIGN_UPDATED : 27|1@1+ (1,0) [0|1] "" XXX
 SG_ DSP_SOFTWARE_VERSION_3RD_NIBBLE : 28|4@1+ (1,0) [0|15] "" XXX
 SG_ FACTORY_ALIGN_STATUS_2 : 32|3@1+ (1,0) [0|7] "" XXX
 SG_ FACTORY_ALIGN_STATUS_1 : 35|3@1+ (1,0) [0|7] "" XXX
 SG_ RECOMMEND_UNCONVERGE : 38|1@1+ (1,0) [0|1] "" XXX
 SG_ FOUND_TARGET : 39|1@1+ (1,0) [0|1] "" XXX
 SG_ FACTORY_MISALIGNMENT_RAW : 40|8@1- (1,0) [-128|127] "raw" XXX
 SG_ SERVICE_ALIGN_UPDATES_DONE : 48|8@1+ (1,0) [0|255] "" XXX
 SG_ VERTICAL_MISALIGNMENT_RAW : 56|8@1- (1,0) [-128|127] "raw" XXX

CM_ BO_ 1248 "Shared Delphi ESR status layout confirmed on sampled 32- and 64-slot Hyundai variants.";
CM_ SG_ 1248 GROUP_COUNTER "2-bit transmit-group counter. It matches 4E1 and 4E3 on sampled 64-slot variants.";
CM_ SG_ 1248 SCAN_INDEX "16-bit radar scan index. It increments once per transmitted group.";
CM_ BO_ 1249 "Delphi ESR Status2 layout confirmed on sampled 64-slot variants only. Sampled 32-slot variants use a different dialect.";
CM_ SG_ 1249 GROUP_COUNTER "Shared 2-bit transmit-group counter on sampled 64-slot variants.";
CM_ BO_ 1250 "Optional BCD software-build timestamp observed on sampled 32-slot radar variants.";
CM_ SG_ 1250 BUILD_YEAR_BCD "Two-digit BCD build year.";
CM_ SG_ 1250 BUILD_MONTH_BCD "BCD build month.";
CM_ SG_ 1250 BUILD_DAY_BCD "BCD build day.";
CM_ SG_ 1250 BUILD_HOUR_BCD "BCD build hour.";
CM_ SG_ 1250 BUILD_MINUTE_BCD "BCD build minute.";
CM_ BO_ 1251 "Shared ESR in-path IDs; low status/mode/alignment fields are confirmed only on 64-slot variants.";
CM_ SG_ 1251 GROUP_COUNTER "Shared 2-bit transmit-group counter on sampled 64-slot variants.";
CM_ SG_ 1251 ACC_MOVING_PATH_ID "One-based target-slot ID. Every nonzero value referenced an active slot across four sampled 32-slot routes.";
CM_ SG_ 1251 CMBB_MOVING_PATH_ID "One-based target-slot ID. Every nonzero value referenced an active slot across four sampled 32-slot routes.";
CM_ SG_ 1251 CMBB_STATIONARY_PATH_ID "One-based target-slot ID. Every nonzero value referenced an active slot across four sampled 32-slot routes.";
CM_ SG_ 1251 FCW_MOVING_PATH_ID "One-based target-slot ID. No nonzero 32-slot sample was available for independent validation.";
CM_ SG_ 1251 FCW_STATIONARY_PATH_ID "One-based target-slot ID. No nonzero 32-slot sample was available for independent validation.";
CM_ SG_ 1251 ACC_STATIONARY_PATH_ID "One-based target-slot ID. Every nonzero value referenced an active slot across four sampled 32-slot routes.";
CM_ BO_ 1252 "Relocated ESR Status5 ADC payload. Names are protocol-derived; raw units are uncalibrated.";
CM_ BO_ 1253 "Relocated ESR Status6 alignment payload. Factory-status bits are route-validated.";
CM_ SG_ 1253 FACTORY_ALIGN_STATUS_1 "Protocol enum: 0 off, 1 busy, 2 success, 3 fail-no-target, 4 fail-deviation-too-large, 5 fail-variance-too-large.";
CM_ SG_ 1253 FACTORY_ALIGN_STATUS_2 "Protocol enum: 0 off, 1 busy, 2 success, 3 fail-no-target, 4 fail-deviation-too-large, 5 fail-variance-too-large.";
  """)

  # The first 32 slots are present on every sampled platform. Older Ioniq,
  # Ioniq PHEV, and K7 radars carry a second, identically packed 32-slot bank.
  for a in range(0x500, 0x500 + 64):
    parts.append(f"""
BO_ {a} RADAR_TRACK_{a:x}: 8 RADAR
 SG_ UNKNOWN_1 : 7|8@0- (1,0) [-128|127] "" XXX
 SG_ ONCOMING_ESR : 0|1@0+ (1,0) [0|1] "" XXX
 SG_ GROUPING_CHANGED_ESR : 1|1@0+ (1,0) [0|1] "" XXX
 SG_ REL_LAT_SPEED_ESR : 7|6@0- (0.25,0) [-8|7.75] "m/s" XXX
 SG_ AZIMUTH : 12|10@0- (0.1,0) [-51.2|51.1] "deg" XXX
 SG_ STATE : 15|3@0+ (1,0) [0|7] "" XXX
 SG_ LONG_DIST : 18|11@0+ (0.1,0) [0|204.7] "m" XXX
 SG_ REL_ACCEL : 33|10@0- (0.02,0) [-10.24|10.22] "m/s^2" XXX
 SG_ REL_ACCEL_ESR : 33|10@0- (0.05,0) [-25.6|25.55] "m/s^2" XXX
 SG_ WIDTH_ESR : 37|4@0+ (0.5,0) [0|7.5] "m" XXX
 SG_ COUNTER : 38|1@0+ (1,0) [0|1] "" XXX
 SG_ BRIDGE_OBJECT_ESR : 39|1@0+ (1,0) [0|1] "" XXX
 SG_ REL_SPEED : 53|14@0- (0.01,0) [-81.92|81.91] "m/s" XXX
 SG_ MED_RANGE_MODE_ESR : 55|2@0+ (1,0) [0|3] "" XXX
    """)

  return {"hyundai_radar_500_53f.dbc": "".join(parts)}
