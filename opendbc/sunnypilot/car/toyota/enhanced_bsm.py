"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.toyota import toyotacan
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

LEFT_BLINDSPOT = b"\x41"
RIGHT_BLINDSPOT = b"\x42"
LEFT_SIDE = LEFT_BLINDSPOT[0]
RIGHT_SIDE = RIGHT_BLINDSPOT[0]

# BLINDSPOTD1/D2 aren't real distances despite the DBC name: verified against a real route, D1 only
# ever reads ~0 (idle) or ~50/52 (left/right occupied), D2 only ~0 or ~27/28 - two near-constant codes,
# not a continuous measurement. Treated as a plain nonzero/occupied flag below, not a magnitude.

# DEBUG also carries a 1-bit BLINDSPOT flag (byte 4) that isn't read here. Checked against the same
# route: stayed 0 across all frames, including every confirmed real detection - not a usable signal.

# no ack on the enable command, so re-send it periodically in case the ECU missed or forgot it
ENABLE_REASSERT_FRAMES = 500  # ~5s @ 100Hz


class EnhancedBsm:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

  @property
  def enabled(self):
    return bool(self.CP_SP.flags & ToyotaFlagsSP.SP_ENHANCED_BSM)


class _BsmSideState:
  def __init__(self):
    self.blindspot = False
    self.counter = 0

  def update(self, distance_1, distance_2):
    # refresh the hold on every valid occupied reading, not just when the value changes - a firmware
    # that reports an unchanging occupied code every poll must not be read as "gone" after 100 frames
    if distance_1 != 0 or distance_2 != 0:
      self.counter = 100

  def decay(self):
    self.counter = max(0, self.counter - 1)
    self.blindspot = self.counter > 0


# Enhanced BSM (@arne182, @rav4kumar)
class EnhancedBsmCarState(EnhancedBsm):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

    self._sides = {LEFT_SIDE: _BsmSideState(), RIGHT_SIDE: _BsmSideState()}

  def update(self, cp, frame: int) -> tuple[bool, bool]:
    if frame <= 199:
      return False, False

    # Let's keep all the commented out code for easy debug purposes in the future.
    distance_1 = cp.vl["DEBUG"].get('BLINDSPOTD1')
    distance_2 = cp.vl["DEBUG"].get('BLINDSPOTD2')
    side = cp.vl["DEBUG"].get('BLINDSPOTSIDE')

    if all(val is not None for val in [distance_1, distance_2, side]) and side in self._sides:
      self._sides[side].update(distance_1, distance_2)

    for side_state in self._sides.values():
      side_state.decay()

    return self._sides[LEFT_SIDE].blindspot, self._sides[RIGHT_SIDE].blindspot


class _BsmSideController:
  def __init__(self, addr_byte: bytes):
    self.addr_byte = addr_byte
    self.debug_enabled = False
    self.last_poll_frame = 0
    self.last_enable_frame = 0

  def update(self, frame: int, poll_phase: int, e_bsm_rate: int, always_on: bool, vego_ok: bool) -> list[CanData]:
    can_sends = []

    if not self.debug_enabled:
      if always_on or vego_ok:  # eagle eye camera will stop working if bsm is switched on under 6m/s
        can_sends.append(toyotacan.create_set_bsm_debug_mode(self.addr_byte, True))
        self.debug_enabled = True
        self.last_enable_frame = frame
        self.last_poll_frame = frame  # give the poll loop a fresh baseline so the stale-poll disable check below can't fire before the first real poll
    else:
      if not always_on and frame - self.last_poll_frame > 50:
        can_sends.append(toyotacan.create_set_bsm_debug_mode(self.addr_byte, False))
        self.debug_enabled = False
      elif frame - self.last_enable_frame > ENABLE_REASSERT_FRAMES:
        can_sends.append(toyotacan.create_set_bsm_debug_mode(self.addr_byte, True))
        self.last_enable_frame = frame

      if frame % e_bsm_rate == poll_phase:
        can_sends.append(toyotacan.create_bsm_polling_status(self.addr_byte))
        self.last_poll_frame = frame

    return can_sends


# Enhanced BSM (@arne182, @rav4kumar)
class EnhancedBsmCarController(EnhancedBsm):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

    self._left = _BsmSideController(LEFT_BLINDSPOT)
    self._right = _BsmSideController(RIGHT_BLINDSPOT)

  def update(self, CS: structs.CarState, frame: int, e_bsm_rate: int = 20, always_on: bool = False) -> list[CanData]:
    if frame <= 200:
      return []

    vego_ok = CS.out.vEgo > 6
    can_sends = self._left.update(frame, 0, e_bsm_rate, always_on, vego_ok)
    can_sends += self._right.update(frame, e_bsm_rate // 2, e_bsm_rate, always_on, vego_ok)
    return can_sends
