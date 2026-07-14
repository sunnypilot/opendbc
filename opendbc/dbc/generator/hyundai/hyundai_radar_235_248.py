#!/usr/bin/env python3
import os

if __name__ == "__main__":
  dbc_name = os.path.basename(__file__).replace(".py", ".dbc")
  hyundai_path = os.path.dirname(os.path.realpath(__file__))
  with open(os.path.join(hyundai_path, dbc_name), "w", encoding='utf-8') as f:
    f.write("""
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

    for a in range(0x235, 0x235 + 20):
        f.write(f"""
BO_ {a} RADAR_TRACK_{a:x}: 32 RADAR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ STATE : 24|4@1+ (1,0) [0|15] "" XXX
 SG_ LONG_DIST : 64|12@1+ (0.05,0) [0|204.75] "m" XXX
 SG_ LAT_DIST : 78|12@1+ (0.05,-102.4) [-102.4|102.35] "m" XXX
 SG_ REL_SPEED : 91|12@1+ (0.05,-100) [-100|104.75] "m/s" XXX
 SG_ NEW_SIGNAL_4 : 104|10@1+ (1,0) [0|1023] "" XXX
 SG_ AZIMUTH : 115|9@1- (0.1,0) [-25.6|25.5] "deg" XXX
 SG_ NEW_SIGNAL_6 : 125|12@1+ (1,0) [0|4095] "" XXX
 SG_ NEW_SIGNAL_7 : 138|12@1+ (1,0) [0|4095] "" XXX
 SG_ NEW_SIGNAL_8 : 151|10@1+ (1,0) [0|1023] "" XXX
 SG_ AZIMUTH_HR : 176|14@1+ (0.0238,-195.05) [-195.05|195.07] "deg" XXX
    """)
