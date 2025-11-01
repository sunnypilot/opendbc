"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car.structs import CarParams
from opendbc.car.honda.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS_EXT = {
  CAR.HONDA_ACCORD: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TVA,A150\x00\x00',
    ],
  },
  CAR.HONDA_CIVIC: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TBA,A030\x00\x00',
    ],
  },
  CAR.HONDA_CIVIC_BOSCH: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TGG,A020\x00\x00',
      b'39990-TGG,A120\x00\x00',
    ],
  },
  CAR.HONDA_CRV_5G: {
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TLA,A040\x00\x00',
    ],
  },
  CAR.HONDA_CLARITY: {
    (Ecu.shiftByWire, 0x18da0bf1, None): [
      b'54008-TRW-A910\x00\x00',
    ],
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TRW-A010\x00\x00',
      b'57114-TRW-A020\x00\x00',
    ],
    (Ecu.eps, 0x18da30f1, None): [
      b'39990-TRW-A020\x00\x00',
      b'39990-TRW,A020\x00\x00',  # modified firmware
      b'39990,TRW,A020\x00\x00',  # extra modified firmware
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TRW-A210\x00\x00',
      b'77959-TRW-A220\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TRW-A010\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'36161-TRW-A110\x00\x00',
    ],
  },
  CAR.HONDA_ACCORD_9G: {
    (Ecu.vsa, 0x18DA28F1, None): [
      b'57114-T2F-X840\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18DAB0F1, None): [
      b'36161-T2F-A140\x00\x00',
      b'36161-T3Z-A830\x00\x00'
    ],
    (Ecu.srs, 0x18DA53F1, None): [
      b'77959-T2F-A030\x00\x00',
      b'77959-T3Z-A020\x00\x00',
    ],
    (Ecu.gateway, 0x18DAEFF1, None): [
      b'38897-T3W-0130\x00\x00',
    ],
  },
}
