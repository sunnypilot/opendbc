"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag

class TeslaFlagsSP(IntFlag):
  """
    Flags for Tesla specific quirks within sunnypilot.
  """
  COOP_STEERING = 1