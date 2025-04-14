"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag
from opendbc.car import make_can_msg
# Blindspot marker bytes
LEFT_BLINDSPOT = b"\x41"
RIGHT_BLINDSPOT = b"\x42"

def create_set_bsm_debug_mode(lr_blindspot, enabled):
  dat = b"\x02\x10\x60\x00\x00\x00\x00" if enabled else b"\x02\x10\x01\x00\x00\x00\x00"
  dat = lr_blindspot + dat
  return make_can_msg(0x750, dat, 0)
def create_bsm_polling_status(lr_blindspot):
  return make_can_msg(0x750, lr_blindspot + b"\x02\x21\x69\x00\x00\x00\x00", 0)

class ToyotaFlagsSP(IntFlag):
  """
  Special flags for Toyota SunnyPilot features
  """
  SP_ENHANCED_BSM = 1

class ToyotaSafetyFlagsSP:
  DEFAULT = 0
  UNSUPPORTED_DSU = 1