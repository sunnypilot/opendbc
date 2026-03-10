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

    for a in range(0x180, 0x180 + 5):
        f.write(f"""
BO_ {a} RADAR_TRACK_{a:x}: 32 RADAR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ 1_LONG : 64|13@1+ (1,0) [0|8191] "" XXX
 SG_ NEW_SIGNAL_2 : 78|12@1+ (1,0) [0|4095] "" XXX
 SG_ NEW_SIGNAL_3 : 91|12@1+ (1,0) [0|4095] "" XXX
 SG_ NEW_SIGNAL_4 : 104|10@1+ (1,0) [0|1023] "" XXX
 SG_ NEW_SIGNAL_5 : 115|9@1- (1,0) [0|511] "" XXX
 SG_ NEW_SIGNAL_6 : 124|16@1+ (1,0) [0|65535] "" XXX
 SG_ 2_LONG : 192|13@1+ (1,0) [0|8191] "" XXX
 SG_ NEW_SIGNAL_12 : 206|12@1+ (1,0) [0|4095] "" XXX
 SG_ NEW_SIGNAL_13 : 219|12@1+ (1,0) [0|4095] "" XXX
 SG_ NEW_SIGNAL_14 : 232|10@1+ (1,0) [0|1023] "" XXX
 SG_ NEW_SIGNAL_8 : 243|9@1- (1,0) [0|511] "" XXX
 SG_ NEW_SIGNAL_9 : 252|4@1+ (1,0) [0|15] "" XXX
    """)
