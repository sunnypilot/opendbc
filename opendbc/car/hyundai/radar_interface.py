import math
from dataclasses import dataclass

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import RadarType

RADAR_TRACK_BUSES = (0, 1, 2)


# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


@dataclass(frozen=True)
class HyundaiRadarTrackSpec:
  name: str
  start_addr: int
  msg_count: int
  track_prefixes: tuple[str, ...]

  @property
  def end_addr(self) -> int:
    return self.start_addr + self.msg_count - 1

  @property
  def address_range(self) -> range:
    return range(self.start_addr, self.end_addr + 1)

  @property
  def dbc_name(self) -> str:
    return f"hyundai_{self.name.lower()}_generated"


@dataclass(frozen=True)
class HyundaiRadarTrackConfig:
  spec: HyundaiRadarTrackSpec
  bus: int


HyundaiRadarTrackKey = tuple[str, int]


@dataclass
class HyundaiRadarParserConfig:
  spec: HyundaiRadarTrackSpec
  bus: int
  parser: CANParser


RADAR_500_51F = HyundaiRadarTrackSpec("RADAR_500_51F", 0x500, 32, ("",))
RADAR_210_21F = HyundaiRadarTrackSpec("RADAR_210_21F", 0x210, 16, ("1_", "2_"))
RADAR_235_248 = HyundaiRadarTrackSpec("RADAR_235_248", 0x235, 20, ("",))
RADAR_3A5_3C4 = HyundaiRadarTrackSpec("RADAR_3A5_3C4", 0x3A5, 32, ("",))
RADAR_602_617 = HyundaiRadarTrackSpec("RADAR_602_617", 0x602, 16, ("1_", "2_"))

HYUNDAI_RADAR_TRACK_SPECS = (RADAR_500_51F, RADAR_210_21F, RADAR_235_248, RADAR_3A5_3C4, RADAR_602_617)
DEFAULT_RADAR_TRIGGER_MSG = max(rt.end_addr for rt in HYUNDAI_RADAR_TRACK_SPECS)
RADAR_TRACK_SPEC_BY_ADDR = {addr: spec for spec in HYUNDAI_RADAR_TRACK_SPECS for addr in spec.address_range}

DETECTED_RADAR_TRACKS: dict[str, tuple[HyundaiRadarTrackConfig, ...]] = {}


def detect_radar_tracks(fingerprint: dict[int, dict[int, int]]) -> tuple[HyundaiRadarTrackConfig, ...]:
  detected: list[HyundaiRadarTrackConfig] = []
  for radar_spec in HYUNDAI_RADAR_TRACK_SPECS:
    for bus in RADAR_TRACK_BUSES:
      if all(addr in fingerprint[bus] for addr in radar_spec.address_range):
        detected.append(HyundaiRadarTrackConfig(radar_spec, bus))
  return tuple(detected)


def set_detected_radar_tracks(car_fingerprint: str, radar_tracks: tuple[HyundaiRadarTrackConfig, ...]) -> None:
  DETECTED_RADAR_TRACKS[car_fingerprint] = radar_tracks


def get_detected_radar_tracks(CP) -> tuple[HyundaiRadarTrackConfig, ...]:
  return DETECTED_RADAR_TRACKS.get(CP.carFingerprint, ())


def get_radar_can_parser(radar_track: HyundaiRadarTrackConfig) -> CANParser:
  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in radar_track.spec.address_range]
  return CANParser(radar_track.spec.dbc_name, messages, radar_track.bus)


def get_track_storage_key(radar_spec: HyundaiRadarTrackSpec, addr: int, track_prefix: str) -> HyundaiRadarTrackKey:
  if radar_spec.name in ("RADAR_500_51F", "RADAR_235_248", "RADAR_3A5_3C4"):
    return (radar_spec.name, addr)

  track_index = int(track_prefix[0]) - 1
  return (radar_spec.name, addr * 2 + track_index)


def get_track_ts_nanos(parser: CANParser, msg_name: str, radar_spec: HyundaiRadarTrackSpec, track_prefix: str) -> int:
  if radar_spec.name == "RADAR_602_617":
    return parser.ts_nanos[msg_name][f"{track_prefix}DISTANCE"]
  if radar_spec.name in ("RADAR_210_21F", "RADAR_235_248"):
    return parser.ts_nanos[msg_name][f"{track_prefix}LONG_DIST"]
  return parser.ts_nanos[msg_name]["LONG_DIST"]


def decode_radar_track(radar_spec: HyundaiRadarTrackSpec, track_msg, track_prefix: str) -> tuple[float, float, float, float]:
  if radar_spec.name == "RADAR_602_617":
    return (
      track_msg[f"{track_prefix}DISTANCE"],
      track_msg[f"{track_prefix}LATERAL"],
      track_msg[f"{track_prefix}SPEED"],
      float("nan"),
    )

  if radar_spec.name == "RADAR_210_21F":
    return (
      track_msg[f"{track_prefix}LONG_DIST"],
      track_msg[f"{track_prefix}LAT_DIST"],
      track_msg[f"{track_prefix}REL_SPEED"],
      float("nan"),
    )

  if radar_spec.name == "RADAR_500_51F":
    azimuth = math.radians(track_msg["AZIMUTH"])
    long_dist = track_msg["LONG_DIST"]
    return (
      math.cos(azimuth) * long_dist,
      0.5 * -math.sin(azimuth) * long_dist,
      track_msg["REL_SPEED"],
      track_msg["REL_ACCEL"],
    )

  if radar_spec.name == "RADAR_235_248":
    return (
      track_msg["LONG_DIST"],
      track_msg["LAT_DIST"],
      track_msg["REL_SPEED"],
      float("nan"),
    )

  return (
    track_msg["LONG_DIST"],
    track_msg["LAT_DIST"],
    track_msg["REL_SPEED"],
    track_msg["REL_ACCEL"],
  )


def is_radar_track_valid(radar_spec: HyundaiRadarTrackSpec, track_msg, track_prefix: str) -> bool:
  if radar_spec.name == "RADAR_602_617":
    return track_msg[f"{track_prefix}DISTANCE"] != 255.75

  if radar_spec.name == "RADAR_210_21F":
    return track_msg[f"{track_prefix}STATE"] in (3, 4)

  if radar_spec.name == "RADAR_235_248":
    return track_msg["STATE"] in (3, 4)

  return track_msg["STATE"] in (3, 4)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.pts: dict[int | HyundaiRadarTrackKey, structs.RadarData.RadarPoint] = {}
    detected_tracks = get_detected_radar_tracks(CP)
    self.radar_tracks_available = bool(detected_tracks)
    self.radar_tracks = detected_tracks
    self.radar_parsers = tuple(HyundaiRadarParserConfig(rt.spec, rt.bus, get_radar_can_parser(rt)) for rt in self.radar_tracks)
    self.seen_radar_addrs = {(rt.spec.name, rt.bus): set(rt.spec.address_range) for rt in detected_tracks}
    self.updated_messages: set[tuple[int, int]] = set()
    self.active_radar_buses: dict[str, int] = {}
    self.trigger_msg = DEFAULT_RADAR_TRIGGER_MSG
    self.track_id = 0

    self.radar_off_can = True
    self.rcp: CANParser | None = None
    self._radar_mode: int | None = None

    if CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR:
      radar_mode = RadarType.FULL_RADAR
    elif CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY:
      radar_mode = RadarType.LEAD_ONLY
    else:
      radar_mode = RadarType.OFF
    self.set_radar_mode(radar_mode)

  def _discover_radar_parsers(self, can_strings) -> None:
    existing_parsers = {(parser.spec.name, parser.bus) for parser in self.radar_parsers}
    updated_candidates: dict[tuple[str, int], HyundaiRadarTrackSpec] = {}
    for _, can_messages in can_strings:
      for raw_addr, _, raw_bus in can_messages:
        addr, bus = int(raw_addr), int(raw_bus)
        spec = RADAR_TRACK_SPEC_BY_ADDR.get(addr)
        if spec is None or bus not in RADAR_TRACK_BUSES:
          continue

        parser_key = (spec.name, bus)
        if parser_key not in existing_parsers:
          self.seen_radar_addrs.setdefault(parser_key, set()).add(addr)
          updated_candidates[parser_key] = spec

    discovered_tracks = [
      HyundaiRadarTrackConfig(spec, bus)
      for (spec_name, bus), spec in updated_candidates.items()
      if all(addr in self.seen_radar_addrs[(spec_name, bus)] for addr in spec.address_range)
    ]

    if not discovered_tracks:
      return

    self.radar_tracks = (*self.radar_tracks, *discovered_tracks)
    self.radar_parsers = (*self.radar_parsers, *(
      HyundaiRadarParserConfig(track.spec, track.bus, get_radar_can_parser(track)) for track in discovered_tracks
    ))
    if self.rcp is None:
      self.rcp = self.radar_parsers[0].parser

  @property
  def radar_mode(self) -> int | None:
    return self._radar_mode

  def set_radar_mode(self, radar_mode: int) -> bool:
    if radar_mode not in (RadarType.OFF, RadarType.LEAD_ONLY, RadarType.FULL_RADAR):
      raise ValueError(f"invalid Hyundai radar mode: {radar_mode}")
    if radar_mode == self._radar_mode:
      return False

    radar_mode_flags = HyundaiFlagsSP.RADAR_OFF | HyundaiFlagsSP.RADAR_LEAD_ONLY | HyundaiFlagsSP.RADAR_FULL_RADAR
    self.CP_SP.flags &= ~radar_mode_flags.value
    mode_flag = {
      RadarType.OFF: HyundaiFlagsSP.RADAR_OFF,
      RadarType.LEAD_ONLY: HyundaiFlagsSP.RADAR_LEAD_ONLY,
      RadarType.FULL_RADAR: HyundaiFlagsSP.RADAR_FULL_RADAR,
    }[radar_mode]
    self.CP_SP.flags |= mode_flag.value

    self.pts.clear()
    self.updated_messages.clear()
    self.active_radar_buses.clear()

    if radar_mode == RadarType.FULL_RADAR:
      self.trigger_msg = max((track.spec.end_addr for track in self.radar_tracks), default=DEFAULT_RADAR_TRIGGER_MSG)
      self.rcp = self.radar_parsers[0].parser if self.radar_parsers else None
      self.radar_off_can = False
    elif radar_mode == RadarType.LEAD_ONLY and self.use_radar_interface_ext:
      self.initialize_radar_ext(DEFAULT_RADAR_TRIGGER_MSG)
      self.radar_off_can = self.rcp is None
    else:
      self.trigger_msg = self.get_trigger_msg(DEFAULT_RADAR_TRIGGER_MSG)
      self.rcp = None
      self.radar_off_can = True

    self._radar_mode = radar_mode
    return True

  def update(self, can_strings):
    if self.radar_off_can:
      ret = super().update(None)
      if ret is not None:
        ret.radarTracksAvailable = self.radar_tracks_available
      return ret

    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY and self.use_radar_interface_ext:
      vls = self.rcp.update(can_strings)
      self.updated_messages.update((0, addr) for addr in vls)

      if (0, self.trigger_msg) not in self.updated_messages:
        return None

      rr = self._update(self.updated_messages)
      self.updated_messages.clear()
      return rr

    self._discover_radar_parsers(can_strings)
    radar_cycle_complete = False
    for radar_parser in self.radar_parsers:
      vls = radar_parser.parser.update(can_strings)
      relevant_addrs = {
        addr for addr in vls
        if radar_parser.spec.start_addr <= addr <= radar_parser.spec.end_addr
      }
      if relevant_addrs:
        self.seen_radar_addrs.setdefault((radar_parser.spec.name, radar_parser.bus), set()).update(relevant_addrs)
        self.updated_messages.update((radar_parser.bus, addr) for addr in relevant_addrs)
        radar_cycle_complete |= radar_parser.spec.end_addr in relevant_addrs

    if not radar_cycle_complete:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()
    return rr

  def _radar_range_complete(self, radar_parser: HyundaiRadarParserConfig) -> bool:
    seen_addrs = self.seen_radar_addrs.get((radar_parser.spec.name, radar_parser.bus), set())
    return all(addr in seen_addrs for addr in radar_parser.spec.address_range)

  @staticmethod
  def _radar_parser_valid_count(radar_parser: HyundaiRadarParserConfig, updated_messages: set[tuple[int, int]]) -> int:
    valid_count = 0
    for addr in radar_parser.spec.address_range:
      if updated_messages and (radar_parser.bus, addr) not in updated_messages:
        continue

      msg_name = f"RADAR_TRACK_{addr:x}"
      msg = radar_parser.parser.vl[msg_name]
      for track_prefix in radar_parser.spec.track_prefixes:
        if get_track_ts_nanos(radar_parser.parser, msg_name, radar_parser.spec, track_prefix) == 0:
          continue
        if is_radar_track_valid(radar_parser.spec, msg, track_prefix):
          valid_count += 1
    return valid_count

  def _select_active_radar_parser(self, radar_parsers: list[HyundaiRadarParserConfig],
                                  updated_messages: set[tuple[int, int]]) -> HyundaiRadarParserConfig:
    valid_counts = {rp.bus: self._radar_parser_valid_count(rp, updated_messages) for rp in radar_parsers}
    current_bus = self.active_radar_buses.get(radar_parsers[0].spec.name)
    current_parser = next((rp for rp in radar_parsers if rp.bus == current_bus), None)

    best_parser = max(
      radar_parsers,
      key=lambda rp: (
        valid_counts[rp.bus],
        rp.bus == current_bus,
      ),
    )
    if valid_counts[best_parser.bus] == 0 and current_parser is not None:
      best_parser = current_parser

    self.active_radar_buses[best_parser.spec.name] = best_parser.bus
    return best_parser

  def _update_radar_points_from_parser(self, radar_parser: HyundaiRadarParserConfig) -> int:
    track_count = 0
    for addr in radar_parser.spec.address_range:
      msg_name = f"RADAR_TRACK_{addr:x}"
      msg = radar_parser.parser.vl[msg_name]
      for track_prefix in radar_parser.spec.track_prefixes:
        track_key = get_track_storage_key(radar_parser.spec, addr, track_prefix)
        if get_track_ts_nanos(radar_parser.parser, msg_name, radar_parser.spec, track_prefix) != 0 and \
           is_radar_track_valid(radar_parser.spec, msg, track_prefix):
          track_count += 1
          if track_key not in self.pts:
            self.pts[track_key] = structs.RadarData.RadarPoint()
            self.pts[track_key].trackId = self.track_id
            self.track_id += 1

          d_rel, y_rel, v_rel, a_rel = decode_radar_track(radar_parser.spec, msg, track_prefix)
          pt = self.pts[track_key]
          pt.measured = True
          pt.dRel = d_rel
          pt.yRel = y_rel
          pt.vRel = v_rel
          pt.aRel = a_rel
          pt.yvRel = float('nan')
        elif track_key in self.pts:
          del self.pts[track_key]
    return track_count

  def _update(self, updated_messages):
    ret = structs.RadarData()
    ret.radarTracksAvailable = self.radar_tracks_available

    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY:
      if self.use_radar_interface_ext and self.rcp is not None:
        return self.update_ext(ret)
      return ret

    active_radar_parsers: list[HyundaiRadarParserConfig] = []
    active_radar_sources: list[tuple[HyundaiRadarParserConfig, int]] = []
    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR:
      radar_parsers_by_spec: dict[str, list[HyundaiRadarParserConfig]] = {}
      for radar_parser in self.radar_parsers:
        if not self._radar_range_complete(radar_parser):
          continue
        self.radar_tracks_available = True
        radar_parsers_by_spec.setdefault(radar_parser.spec.name, []).append(radar_parser)

      ret.radarTracksAvailable = self.radar_tracks_available

      for radar_parsers in radar_parsers_by_spec.values():
        active_radar_parser = self._select_active_radar_parser(radar_parsers, updated_messages)
        active_radar_parsers.append(active_radar_parser)
        track_count = self._update_radar_points_from_parser(active_radar_parser)
        active_radar_sources.append((active_radar_parser, track_count))

    if any(not radar_parser.parser.can_valid for radar_parser in active_radar_parsers):
      ret.errors.canError = True

    ret.trackSources = [{
      "startAddress": radar_parser.spec.start_addr,
      "endAddress": radar_parser.spec.end_addr,
      "bus": radar_parser.bus,
      "trackCount": track_count,
    } for radar_parser, track_count in active_radar_sources]
    ret.points = list(self.pts.values())
    return ret
