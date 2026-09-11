import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32
CANFD_RADAR_START_ADDR = 0x210
CANFD_RADAR_MSG_COUNT = 16
CANFD_RADAR_TRACKS_PER_MSG = 2
CANFD_RADAR_CONFIRMED_AGE = 11
CANFD_RADAR_CORROBORATED_AGE = 2
CANFD_RADAR_SCC_MAX_DREL_DELTA = 2.0
CANFD_RADAR_SCC_MAX_VREL_DELTA = 2.0
CANFD_RADAR_SCC_FALLBACK_KEY = CANFD_RADAR_MSG_COUNT * CANFD_RADAR_TRACKS_PER_MSG
CANFD_RADAR_SCC_MAX_AGE_NS = 100_000_000

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
    self.rcp_scc = None
    self.scc_seen = False

    # Group 1 tracks take roughly half a second to reach the normal age gate.
    # Keep the camera-SCC lead as an independent confidence source so a matching
    # young track can be admitted early without relaxing radar-only validation.
    if self.use_canfd_radar_tracks and CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
      self.rcp_scc = self.get_radar_ext_can_parser()

    if self.rcp is None:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    if self.rcp_scc is not None:
      self.scc_seen |= 0x1A0 in self.rcp_scc.update(can_strings)

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

  def _get_canfd_scc_lead(self):
    if self.rcp_scc is None or not self.scc_seen or not self.rcp_scc.can_valid:
      return None

    radar_ts = self.rcp.ts_nanos[self.trigger_msg]["VALID_CNT2"]
    scc_ts = self.rcp_scc.ts_nanos["SCC_CONTROL"]["ACC_ObjDist"]
    if abs(radar_ts - scc_ts) > CANFD_RADAR_SCC_MAX_AGE_NS:
      return None

    msg = self.rcp_scc.vl["SCC_CONTROL"]
    if msg["ACC_ObjDist"] >= 204.6:
      return None

    return msg["ACC_ObjDist"], msg["ACC_ObjRelSpd"]

  @staticmethod
  def _canfd_track_matches_scc(track, scc_lead):
    return (abs(track["dRel"] - scc_lead[0]) <= CANFD_RADAR_SCC_MAX_DREL_DELTA and
            abs(track["vRel"] - scc_lead[1]) <= CANFD_RADAR_SCC_MAX_VREL_DELTA)

  def _update_canfd_radar_tracks(self):
    tracks = []
    for bank in range(1, CANFD_RADAR_TRACKS_PER_MSG + 1):
      for slot, addr in enumerate(range(CANFD_RADAR_START_ADDR, CANFD_RADAR_START_ADDR + CANFD_RADAR_MSG_COUNT)):
        point_key = (bank - 1) * CANFD_RADAR_MSG_COUNT + slot
        msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
        tracks.append({
          "point_key": point_key,
          "age": int(msg[f"VALID_CNT{bank}"]),
          "dRel": msg[f"LONG_DIST{bank}"],
          "yRel": msg[f"LAT_DIST{bank}"],
          "vRel": msg[f"REL_SPEED{bank}"],
          "yvRel": msg[f"LAT_SPEED{bank}"],
          "aRel": msg[f"REL_ACCEL{bank}"],
        })

    scc_lead = self._get_canfd_scc_lead()
    scc_match = None
    if scc_lead is not None:
      candidates = [track for track in tracks if track["age"] > 0 and self._canfd_track_matches_scc(track, scc_lead)]
      if candidates:
        scc_match = min(candidates, key=lambda track: (
          abs(track["dRel"] - scc_lead[0]) / CANFD_RADAR_SCC_MAX_DREL_DELTA +
          abs(track["vRel"] - scc_lead[1]) / CANFD_RADAR_SCC_MAX_VREL_DELTA
        ))

    for track in tracks:
      point_key = track["point_key"]
      confirmed = track["age"] >= CANFD_RADAR_CONFIRMED_AGE
      corroborated = track is scc_match and track["age"] >= CANFD_RADAR_CORROBORATED_AGE
      valid = confirmed or corroborated

      if not valid:
        self.pts.pop(point_key, None)
        continue

      if point_key not in self.pts:
        self.pts[point_key] = structs.RadarData.RadarPoint()
        self.pts[point_key].trackId = self.track_id
        self.track_id += 1

      point = self.pts[point_key]
      point.dRel = track["dRel"]
      point.yRel = track["yRel"]
      point.vRel = track["vRel"]
      point.deprecated.aRel = track["aRel"]
      point.deprecated.yvRel = track["yvRel"]
      point.deprecated.measured = True

    # Preserve the existing camera-SCC behavior as a temporary fallback when
    # no raw track corroborates its lead. NaN lateral position prevents this
    # virtual point from participating in radar-only low-speed override.
    if scc_lead is not None and (scc_match is None or scc_match["age"] < CANFD_RADAR_CORROBORATED_AGE):
      if CANFD_RADAR_SCC_FALLBACK_KEY not in self.pts:
        self.pts[CANFD_RADAR_SCC_FALLBACK_KEY] = structs.RadarData.RadarPoint()
        self.pts[CANFD_RADAR_SCC_FALLBACK_KEY].trackId = self.track_id
        self.track_id += 1

      point = self.pts[CANFD_RADAR_SCC_FALLBACK_KEY]
      point.dRel = scc_lead[0]
      point.yRel = float("nan")
      point.vRel = scc_lead[1]
      point.deprecated.aRel = float("nan")
      point.deprecated.yvRel = 0.0
      point.deprecated.measured = False
    else:
      self.pts.pop(CANFD_RADAR_SCC_FALLBACK_KEY, None)
