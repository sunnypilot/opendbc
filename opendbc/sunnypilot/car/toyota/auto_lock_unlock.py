"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

automatic door locking and unlocking logic (@dragonpilot-community)
thanks to AlexandreSato & cydia2020
https://github.com/AlexandreSato/animalpilot/blob/personal/doors.py
"""

from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP

GearShifter = structs.CarState.GearShifter

UNLOCK_CMD = b'\x40\x05\x30\x11\x00\x40\x00\x00'
LOCK_CMD = b'\x40\x05\x30\x11\x00\x80\x00\x00'
LOCK_AT_SPEED = 10 * CV.KPH_TO_MS


class AutoLockController:
  def __init__(self, CP_SP):
    self.auto_lock_enabled = CP_SP.flags & ToyotaFlagsSP.AUTO_LOCK.value
    self.auto_unlock_enabled = CP_SP.flags & ToyotaFlagsSP.AUTO_UNLOCK.value
    self.last_gear = GearShifter.park
    self.lock_once = False

  def update(self, CS) -> list[CanData]:
    can_sends = []

    if not self.auto_lock_enabled and not self.auto_unlock_enabled:
      return can_sends

    current_gear = CS.out.gearShifter
    if self.last_gear != current_gear and current_gear == GearShifter.park:
      # Auto unlock when shifting to Park
      if self.auto_unlock_enabled:
        can_sends.append(CanData(0x750, UNLOCK_CMD, 0))

      if self.auto_lock_enabled:
        self.lock_once = False

    elif self.auto_lock_enabled:
      # Conditions for auto lock:
      # - In Drive gear
      # - Doors are closed
      # - Haven't locked yet this drive cycle
      # - Speed is above threshold
      if (current_gear == GearShifter.drive and not CS.out.doorOpen and not self.lock_once and CS.out.vEgo >= LOCK_AT_SPEED):
        can_sends.append(CanData(0x750, LOCK_CMD, 0))
        self.lock_once = True

    self.last_gear = current_gear
    return can_sends

  def reset(self):
    self.last_gear = GearShifter.park
    self.lock_once = False