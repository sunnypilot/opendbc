"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

LEFT_BLINDSPOT = b"\x41"
RIGHT_BLINDSPOT = b"\x42"
LEFT_SIDE = LEFT_BLINDSPOT[0]
RIGHT_SIDE = RIGHT_BLINDSPOT[0]

# BLINDSPOTD1/D2 are zone codes: 0 when idle, ~46-53 for a detection; lower values are noise.
BLINDSPOT_NOISE_FLOOR = 35

# Drop the session below 6 m/s (it breaks the eagle-eye camera); re-enable above 7 m/s to avoid toggling.
MIN_BSM_DEBUG_SPEED = 6  # m/s
BSM_DEBUG_ENABLE_SPEED = 7  # m/s
E_BSM_RATE = 20  # frames between polls


def create_set_bsm_debug_mode(addr_byte: bytes, enabled: bool) -> CanData:
  dat = b"\x02\x10\x60\x00\x00\x00\x00" if enabled else b"\x02\x10\x01\x00\x00\x00\x00"
  return CanData(0x750, addr_byte + dat, 0)


def create_bsm_polling_status(addr_byte: bytes) -> CanData:
  return CanData(0x750, addr_byte + b"\x02\x21\x69\x00\x00\x00\x00", 0)


class EnhancedBsm:
  def __init__(self, CP_SP: structs.CarParamsSP):
    self.CP_SP = CP_SP

  @property
  def enabled(self):
    return bool(self.CP_SP.flags & ToyotaFlagsSP.ENHANCED_BSM)


class _BsmSideState:
  def __init__(self):
    self.blindspot = False
    self.counter = 0

  def update(self, distance_1, distance_2):
    self.blindspot = distance_1 > BLINDSPOT_NOISE_FLOOR or distance_2 > BLINDSPOT_NOISE_FLOOR
    self.counter = 100

  def decay(self):
    self.counter = max(0, self.counter - 1)
    if self.counter == 0:
      self.blindspot = False


class EnhancedBsmCarState(EnhancedBsm):
  def __init__(self, CP_SP: structs.CarParamsSP):
    super().__init__(CP_SP)

    self._sides = {LEFT_SIDE: _BsmSideState(), RIGHT_SIDE: _BsmSideState()}
    self._last_timestamp = 0

  def update(self, cp) -> tuple[bool, bool]:
    timestamp = cp.ts_nanos["DEBUG"]["BLINDSPOTSIDE"]
    if timestamp != self._last_timestamp:
      self._last_timestamp = timestamp
      side = cp.vl["DEBUG"]["BLINDSPOTSIDE"]
      if side in self._sides:
        self._sides[side].update(cp.vl["DEBUG"]["BLINDSPOTD1"], cp.vl["DEBUG"]["BLINDSPOTD2"])

    for side_state in self._sides.values():
      side_state.decay()

    return self._sides[LEFT_SIDE].blindspot, self._sides[RIGHT_SIDE].blindspot


class _BsmSideController:
  def __init__(self, addr_byte: bytes):
    self.addr_byte = addr_byte
    self.debug_enabled = False
    self.last_poll_frame = 0

  def update(self, frame: int, poll_phase: int, vego: float) -> list[CanData]:
    can_sends = []

    vego_ok = vego > (MIN_BSM_DEBUG_SPEED if self.debug_enabled else BSM_DEBUG_ENABLE_SPEED)

    if vego_ok:
      if not self.debug_enabled:
        can_sends.append(create_set_bsm_debug_mode(self.addr_byte, True))
        self.debug_enabled = True
        self.last_poll_frame = frame
      if frame % E_BSM_RATE == poll_phase:
        can_sends.append(create_bsm_polling_status(self.addr_byte))
        self.last_poll_frame = frame
      return can_sends

    if self.debug_enabled and frame - self.last_poll_frame > 50:
      can_sends.append(create_set_bsm_debug_mode(self.addr_byte, False))
      self.debug_enabled = False

    return can_sends


class EnhancedBsmCarController(EnhancedBsm):
  def __init__(self, CP_SP: structs.CarParamsSP):
    super().__init__(CP_SP)

    self._left = _BsmSideController(LEFT_BLINDSPOT)
    self._right = _BsmSideController(RIGHT_BLINDSPOT)

  def update(self, CS: structs.CarState, frame: int) -> list[CanData]:
    if frame <= 200:
      return []

    can_sends = self._left.update(frame, E_BSM_RATE // 4, CS.out.vEgo)
    can_sends += self._right.update(frame, 3 * E_BSM_RATE // 4, CS.out.vEgo)
    return can_sends
