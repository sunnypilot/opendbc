#!/usr/bin/env python3
import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.mazda.values import DBC

# 6 radar tracks on CAN IDs 0x361-0x366 (865-870), all at 10 Hz on bus 0.
# Each track reports distance, angle, and relative velocity.
# Empty slots are identified by sentinel values in all three fields.
# 0x361-0x364: stationary + moving objects — RELV_OBJ reliable
# 0x365-0x366: likely moving-vehicle-only slots — RELV_OBJ uses a different
#              encoding (not yet decoded).  These are excluded because without
#              a valid vRel, radard's Kalman filter and lead-matching cannot
#              track them, and NaN velocity would poison downstream consumers.
RADAR_TRACK_ADDRS = list(range(0x361, 0x367))
RADAR_USABLE_ADDRS = set(range(0x361, 0x365))  # only tracks with reliable RELV
RADAR_TRIGGER_MSG = RADAR_TRACK_ADDRS[-1]  # 0x366 — last in the burst
SENTINEL_DIST = 4095 * 0.0625   # 255.9375 m — raw 4095
SENTINEL_ANG = 2046 * 0.015625  # 31.96875 deg — raw 2046
SENTINEL_RELV = -16 * 0.0625    # -1.0 m/s — raw -16


def _create_radar_can_parser(car_fingerprint):
  messages = [(f"RADAR_TRACK_{addr:x}", 10) for addr in RADAR_TRACK_ADDRS]
  return CANParser(DBC[car_fingerprint][Bus.radar], messages, 0)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.track_id = 0
    self.updated_messages = set()
    self.trigger_msg = RADAR_TRIGGER_MSG
    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP.carFingerprint)

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()
    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True

    for addr in RADAR_TRACK_ADDRS:
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      dist = msg['DIST_OBJ']
      ang = msg['ANG_OBJ']
      relv = msg['RELV_OBJ']

      # Empty slots have all three fields set to sentinel values.
      # Also skip tracks whose RELV encoding is not yet decoded — without
      # a valid vRel the downstream Kalman filter and MPC solver break.
      if dist == SENTINEL_DIST or ang == SENTINEL_ANG or relv == SENTINEL_RELV \
         or addr not in RADAR_USABLE_ADDRS:
        if addr in self.pts:
          del self.pts[addr]
        continue

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      azimuth = math.radians(ang)
      self.pts[addr].dRel = math.cos(azimuth) * dist
      self.pts[addr].yRel = -math.sin(azimuth) * dist
      self.pts[addr].vRel = relv
      self.pts[addr].aRel = float('nan')
      self.pts[addr].yvRel = float('nan')
      self.pts[addr].measured = True

    ret.points = list(self.pts.values())
    return ret
