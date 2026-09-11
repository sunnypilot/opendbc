import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import DBC

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32
CANFD_RADAR_START_ADDR = 0x210
CANFD_RADAR_MSG_COUNT = 16
CANFD_RADAR_TRACKS_PER_MSG = 2

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP, CP_SP):
  if CP_SP.flags & HyundaiFlagsSP.CANFD_RADAR_TRACKS:
    messages = [(f"RADAR_TRACK_{addr:x}", 20)
                for addr in range(CANFD_RADAR_START_ADDR, CANFD_RADAR_START_ADDR + CANFD_RADAR_MSG_COUNT)]
    return CANParser("hyundai_canfd_radar_generated", messages, CanBus(CP).ACAN)

  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.updated_messages = set()
    self.use_canfd_radar_tracks = bool(CP_SP.flags & HyundaiFlagsSP.CANFD_RADAR_TRACKS)
    self.trigger_msg = ((CANFD_RADAR_START_ADDR + CANFD_RADAR_MSG_COUNT - 1) if self.use_canfd_radar_tracks
                        else (RADAR_START_ADDR + RADAR_MSG_COUNT - 1))

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP, CP_SP)

    if self.rcp is None:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
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
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.use_canfd_radar_tracks:
      self._update_canfd_radar_tracks()
      ret.points = list(self.pts.values())
      return ret

    if self.use_radar_interface_ext:
      return self.update_ext(ret)

    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret

  def _update_canfd_radar_tracks(self):
    for bank in range(1, CANFD_RADAR_TRACKS_PER_MSG + 1):
      for slot, addr in enumerate(range(CANFD_RADAR_START_ADDR, CANFD_RADAR_START_ADDR + CANFD_RADAR_MSG_COUNT)):
        point_key = (bank - 1) * CANFD_RADAR_MSG_COUNT + slot
        msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
        valid = msg[f"VALID_CNT{bank}"] > 10

        if not valid:
          self.pts.pop(point_key, None)
          continue

        if point_key not in self.pts:
          self.pts[point_key] = structs.RadarData.RadarPoint()
          self.pts[point_key].trackId = self.track_id
          self.track_id += 1

        point = self.pts[point_key]
        point.dRel = msg[f"LONG_DIST{bank}"]
        point.yRel = msg[f"LAT_DIST{bank}"]
        point.vRel = msg[f"REL_SPEED{bank}"]
