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
  CAR.ACURA_MDX_3G: {
    (Ecu.vsa, 0x18da28f1, None): [
      b'57114-TRX-H130\x00\x00',
      b'57114-TZ6-A910\x00\x00',
    ],
    (Ecu.fwdRadar, 0x18dab0f1, None): [
      b'36161-TZ6-A730\x00\x00',
      b'36161-TRX-A820\x00\x00',
    ],
    (Ecu.shiftByWire, 0x18da0bf1, None): [
      b'54008-TRX-A710\x00\x00',
      b'54008-TZ5-A911\x00\x00',
      b'54008-TZ5-A910\x00\x00',
    ],
    (Ecu.srs, 0x18da53f1, None): [
      b'77959-TRX-A011\x00\x00',
      b'77959-TZ5-A110\x00\x00',
      b'77959-TZ5-A220\x00\x00',
    ],
    (Ecu.gateway, 0x18daeff1, None): [
      b'38897-TRX-A220\x00\x00',
    ],
    (Ecu.transmission, 0x18da1ef1, None): [
      b'28101-5NC-A310\x00\x00',
      b'28101-5NC-A770\x00\x00',
    ],
  },
}
