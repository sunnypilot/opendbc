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

    for a in range(0x238, 0x255, 3):
        f.write(f"""
BO_ {a} RADAR_TRACK_{a:x}: 8 RADAR
 SG_ LONG : 6|7@0+ (1,0) [0|127] "" XXX
 SG_ STATE : 9|2@0+ (1,0) [0|3] "" XXX
 SG_ LAT : 55|8@0- (1,0) [0|255] "" XXX

BO_ {a+1} RADAR_TRACK_{a+1:x}: 8 RADAR
 SG_ NEW_SIGNAL_1 : 6|10@1+ (1,0) [0|1023] "" XXX
 SG_ NEW_SIGNAL_2 : 16|10@1+ (1,0) [0|1023] "" XXX
 SG_ NEW_SIGNAL_4 : 42|2@1+ (1,0) [0|3] "" XXX
 SG_ NEW_SIGNAL_3 : 44|12@1- (1,0) [0|4095] "" XXX

BO_ {a+2} RADAR_TRACK_{a+2:x}: 8 RADAR
 SG_ NEW_SIGNAL_6 : 10|6@1+ (1,0) [0|63] "" XXX
 SG_ COUNTER : 31|8@0+ (1,0) [0|255] "" XXX
 SG_ NEW_SIGNAL_1 : 32|12@1+ (1,0) [0|4095] "" XXX
 SG_ NEW_SIGNAL_3 : 45|7@1+ (1,0) [0|127] "" XXX
 SG_ NEW_SIGNAL_4 : 54|4@1+ (1,0) [0|15] "" XXX
 SG_ NEW_SIGNAL_5 : 63|6@0+ (1,0) [0|63] "" XXX
    """)
