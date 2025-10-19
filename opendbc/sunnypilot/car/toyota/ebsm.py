"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.toyota import toyotacan
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

LEFT_BLINDSPOT = b"\x41"
RIGHT_BLINDSPOT = b"\x42"


class EnhancedBSMController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    # Blindspot monitoring state tracking
    self.left_blindspot_debug_enabled = False
    self.right_blindspot_debug_enabled = False
    self.left_last_blindspot_frame = 0
    self.right_last_blindspot_frame = 0

    # Detection tracking variables
    self._left_blindspot = False
    self._left_blindspot_d1 = 0
    self._left_blindspot_d2 = 0
    self._left_blindspot_counter = 0

    self._right_blindspot = False
    self._right_blindspot_d1 = 0
    self._right_blindspot_d2 = 0
    self._right_blindspot_counter = 0

  @property
  def enabled(self):
    return bool(self.CP_SP.flags & ToyotaFlagsSP.SP_ENHANCED_BSM)

  def create_messages(self, CS: structs.CarState, frame: int, e_bsm_rate: int = 20, always_on: bool = True):
    if not self.enabled:
      return []

    can_sends = []

    # Process Left Blindspot
    can_sends.extend(self._process_side_blindspot(
      CS,
      LEFT_BLINDSPOT,
      frame,
      is_left=True,
      e_bsm_rate=e_bsm_rate,
      always_on=always_on
    ))

    # Process Right Blindspot
    can_sends.extend(self._process_side_blindspot(
      CS,
      RIGHT_BLINDSPOT,
      frame,
      is_left=False,
      e_bsm_rate=e_bsm_rate,
      always_on=always_on
    ))

    return can_sends

  def _process_side_blindspot(self, CS, side_marker, frame, is_left=True,
                              min_speed=6.0, e_bsm_rate=20, always_on=True):
    can_sends = []

    # Get the appropriate state variables
    if is_left:
      debug_enabled = self.left_blindspot_debug_enabled
      last_frame = self.left_last_blindspot_frame
    else:
      debug_enabled = self.right_blindspot_debug_enabled
      last_frame = self.right_last_blindspot_frame

    # Activate BSM debug mode
    if not debug_enabled:
      if always_on or CS.vEgo > min_speed:
        can_sends.append(toyotacan.create_set_bsm_debug_mode(side_marker, True))
        if is_left:
          self.left_blindspot_debug_enabled = True
        else:
          self.right_blindspot_debug_enabled = True
    else:
      # Deactivate if not always on and no recent activity
      if not always_on and frame - last_frame > 50:
        can_sends.append(toyotacan.create_set_bsm_debug_mode(side_marker, False))
        if is_left:
          self.left_blindspot_debug_enabled = False
        else:
          self.right_blindspot_debug_enabled = False

      # Polling logic - alternate between left and right
      poll_condition = (is_left and frame % e_bsm_rate == 0) or \
                       (not is_left and frame % e_bsm_rate == e_bsm_rate // 2)

      if poll_condition:
        can_sends.append(toyotacan.create_bsm_polling_status(side_marker))
        if is_left:
          self.left_last_blindspot_frame = frame
        else:
          self.right_last_blindspot_frame = frame

    return can_sends

  def parse_bsm_status(self, cp):
    distance_1 = cp.vl["DEBUG"].get('BLINDSPOTD1')
    distance_2 = cp.vl["DEBUG"].get('BLINDSPOTD2')
    side = cp.vl["DEBUG"].get('BLINDSPOTSIDE')

    if all(val is not None for val in [distance_1, distance_2, side]):
      if side == 65:  # left blind spot
        if distance_1 != self._left_blindspot_d1 or distance_2 != self._left_blindspot_d2:
          self._left_blindspot_d1 = distance_1
          self._left_blindspot_d2 = distance_2
          self._left_blindspot_counter = 100
        self._left_blindspot = distance_1 > 10 or distance_2 > 10

      elif side == 66:  # right blind spot
        if distance_1 != self._right_blindspot_d1 or distance_2 != self._right_blindspot_d2:
          self._right_blindspot_d1 = distance_1
          self._right_blindspot_d2 = distance_2
          self._right_blindspot_counter = 100
        self._right_blindspot = distance_1 > 10 or distance_2 > 10

      # Update counters
      self._left_blindspot_counter = max(0, self._left_blindspot_counter - 1)
      self._right_blindspot_counter = max(0, self._right_blindspot_counter - 1)

      # Reset blind spot status if counter reaches 0
      if self._left_blindspot_counter == 0:
        self._left_blindspot = False
        self._left_blindspot_d1 = self._left_blindspot_d2 = 0

      if self._right_blindspot_counter == 0:
        self._right_blindspot = False
        self._right_blindspot_d1 = self._right_blindspot_d2 = 0

    return self._left_blindspot, self._right_blindspot