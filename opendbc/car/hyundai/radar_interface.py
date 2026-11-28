import math
from collections import deque
from dataclasses import dataclass, field

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, RadarType

RADAR_TRACK_BUSES = (0, 1, 2)
RADAR_CADENCE_TOLERANCE = 0.25
RADAR_FRESH_CYCLE_PERIODS = 3
RADAR_DISCOVERY_INTERVAL = 200
RADAR_PUBLISH_INTERVAL_NS = 45_000_000  # ~20 Hz to match radard/modelV2


# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


@dataclass(frozen=True)
class HyundaiRadarTrackSpec:
  name: str
  start_addr: int
  msg_count: int
  track_prefixes: tuple[str, ...]
  signals: tuple[str, ...]
  frequency: int
  message_size: int
  source_kind: str = "radar"
  required_msg_count: int | None = None

  @property
  def end_addr(self) -> int:
    return self.start_addr + self.msg_count - 1

  @property
  def address_range(self) -> range:
    return range(self.start_addr, self.end_addr + 1)

  @property
  def required_end_addr(self) -> int:
    return self.start_addr + (self.required_msg_count or self.msg_count) - 1

  @property
  def required_address_range(self) -> range:
    return range(self.start_addr, self.required_end_addr + 1)

  @property
  def dbc_name(self) -> str:
    return f"hyundai_{self.name.lower()}_generated"


@dataclass(frozen=True)
class HyundaiRadarTrackConfig:
  spec: HyundaiRadarTrackSpec
  bus: int
  end_addr: int

  @property
  def address_range(self) -> range:
    return range(self.spec.start_addr, self.end_addr + 1)


HyundaiRadarTrackKey = tuple[str, int]


@dataclass
class HyundaiRadarParserConfig:
  spec: HyundaiRadarTrackSpec
  bus: int
  end_addr: int
  parser: CANParser
  cycle_timestamps: deque[int] = field(default_factory=lambda: deque(maxlen=100))
  track_entries: tuple[tuple[int, str, HyundaiRadarTrackKey, str, dict, dict], ...] = field(init=False)

  def __post_init__(self) -> None:
    spec = self.spec
    entries = []
    for addr in range(spec.start_addr, self.end_addr + 1):
      msg_name = f"RADAR_TRACK_{addr:x}"
      vl_msg = self.parser.vl[msg_name]
      ts_msg = self.parser.ts_nanos[msg_name]
      for track_prefix in spec.track_prefixes:
        track_key = get_track_storage_key(spec, addr, track_prefix)
        if spec.name == "RADAR_602_617":
          ts_sig = f"{track_prefix}DISTANCE"
        elif spec.name in ("RADAR_210_21F", "RADAR_235_248"):
          ts_sig = f"{track_prefix}LONG_DIST"
        else:
          ts_sig = "LONG_DIST"
        entries.append((addr, track_prefix, track_key, ts_sig, vl_msg, ts_msg))
    self.track_entries = tuple(entries)

  @property
  def address_range(self) -> range:
    return range(self.spec.start_addr, self.end_addr + 1)

  @property
  def msg_count(self) -> int:
    return self.end_addr - self.spec.start_addr + 1


RADAR_500_53F = HyundaiRadarTrackSpec(
  "RADAR_500_53F", 0x500, 64, ("",),
  ("STATE", "ONCOMING_ESR", "AZIMUTH", "LONG_DIST", "REL_SPEED", "REL_LAT_SPEED_ESR", "REL_ACCEL", "REL_ACCEL_ESR"),
  frequency=20, message_size=8, required_msg_count=32,
)
# Compatibility alias for code importing the former truncated family name.
RADAR_500_51F = RADAR_500_53F
RADAR_210_21F = HyundaiRadarTrackSpec(
  "RADAR_210_21F", 0x210, 16, ("1_", "2_"),
  ("1_STATE", "1_STATE_ALT", "1_MOTION_STATE", "1_AGE", "1_LONG_DIST", "1_LAT_DIST", "1_REL_SPEED", "1_REL_ACCEL",
   "2_STATE", "2_STATE_ALT", "2_MOTION_STATE", "2_AGE", "2_LONG_DIST", "2_LAT_DIST", "2_REL_SPEED", "2_REL_ACCEL"),
  frequency=20, message_size=32,
)
RADAR_235_248 = HyundaiRadarTrackSpec(
  "RADAR_235_248", 0x235, 20, ("",),
  ("QUALITY", "AGE", "MOTION_STATE", "OBJECT_ID", "LONG_DIST", "LAT_DIST", "REL_SPEED", "REL_LAT_SPEED", "REL_ACCEL"),
  frequency=33, message_size=32, source_kind="camera",
)
RADAR_3A5_3C4 = HyundaiRadarTrackSpec(
  "RADAR_3A5_3C4", 0x3A5, 32, ("",), ("STATE", "MOTION_STATE", "AGE", "LONG_DIST", "LAT_DIST", "REL_SPEED", "REL_ACCEL"),
  frequency=20, message_size=24,
)
RADAR_602_617 = HyundaiRadarTrackSpec(
  "RADAR_602_617", 0x602, 16, ("1_", "2_"), ("1_DISTANCE", "1_LATERAL", "1_SPEED", "2_DISTANCE", "2_LATERAL", "2_SPEED"),
  frequency=10, message_size=8,
)

HYUNDAI_RADAR_TRACK_SPECS = (RADAR_500_53F, RADAR_210_21F, RADAR_235_248, RADAR_3A5_3C4, RADAR_602_617)
DEFAULT_RADAR_TRIGGER_MSG = max(rt.end_addr for rt in HYUNDAI_RADAR_TRACK_SPECS)
RADAR_TRACK_SPEC_BY_ADDR = {addr: spec for spec in HYUNDAI_RADAR_TRACK_SPECS for addr in spec.address_range}

DETECTED_RADAR_TRACKS: dict[str, tuple[HyundaiRadarTrackConfig, ...]] = {}


def detect_radar_tracks(fingerprint: dict[int, dict[int, int]]) -> tuple[HyundaiRadarTrackConfig, ...]:
  detected: list[HyundaiRadarTrackConfig] = []
  for radar_spec in HYUNDAI_RADAR_TRACK_SPECS:
    for bus in RADAR_TRACK_BUSES:
      bus_fingerprint = fingerprint.get(bus, {})
      if isinstance(bus_fingerprint, dict):
        required_range_matches = all(bus_fingerprint.get(addr) == radar_spec.message_size for addr in radar_spec.required_address_range)
        full_range_matches = all(bus_fingerprint.get(addr) == radar_spec.message_size for addr in radar_spec.address_range)
      else:
        required_range_matches = all(addr in bus_fingerprint for addr in radar_spec.required_address_range)
        full_range_matches = all(addr in bus_fingerprint for addr in radar_spec.address_range)
      if required_range_matches:
        end_addr = radar_spec.end_addr if full_range_matches else radar_spec.required_end_addr
        detected.append(HyundaiRadarTrackConfig(radar_spec, bus, end_addr))
  return tuple(detected)


def set_detected_radar_tracks(car_fingerprint: str, radar_tracks: tuple[HyundaiRadarTrackConfig, ...]) -> None:
  DETECTED_RADAR_TRACKS[car_fingerprint] = radar_tracks


def get_detected_radar_tracks(CP) -> tuple[HyundaiRadarTrackConfig, ...]:
  return DETECTED_RADAR_TRACKS.get(CP.carFingerprint, ())


def get_radar_can_parser(radar_track: HyundaiRadarTrackConfig) -> CANParser:
  messages = [(f"RADAR_TRACK_{addr:x}", radar_track.spec.frequency) for addr in radar_track.address_range]
  return CANParser(radar_track.spec.dbc_name, messages, radar_track.bus, signals=set(radar_track.spec.signals), track_all_vals=False)


def get_track_storage_key(radar_spec: HyundaiRadarTrackSpec, addr: int, track_prefix: str) -> HyundaiRadarTrackKey:
  if radar_spec.name in ("RADAR_500_53F", "RADAR_235_248", "RADAR_3A5_3C4"):
    return (radar_spec.name, addr)

  track_index = int(track_prefix[0]) - 1
  return (radar_spec.name, addr * 2 + track_index)


def get_track_ts_nanos(parser: CANParser, msg_name: str, radar_spec: HyundaiRadarTrackSpec, track_prefix: str) -> int:
  if radar_spec.name == "RADAR_602_617":
    return parser.ts_nanos[msg_name][f"{track_prefix}DISTANCE"]
  if radar_spec.name in ("RADAR_210_21F", "RADAR_235_248"):
    return parser.ts_nanos[msg_name][f"{track_prefix}LONG_DIST"]
  return parser.ts_nanos[msg_name]["LONG_DIST"]


def decode_radar_track(radar_spec: HyundaiRadarTrackSpec, track_msg, track_prefix: str) -> tuple[float, float, float]:
  if radar_spec.name == "RADAR_602_617":
    return (
      track_msg[f"{track_prefix}DISTANCE"],
      track_msg[f"{track_prefix}LATERAL"],
      track_msg[f"{track_prefix}SPEED"],
    )

  if radar_spec.name == "RADAR_210_21F":
    return (
      track_msg[f"{track_prefix}LONG_DIST"],
      track_msg[f"{track_prefix}LAT_DIST"],
      track_msg[f"{track_prefix}REL_SPEED"],
    )

  if radar_spec.name == "RADAR_500_53F":
    azimuth = math.radians(track_msg["AZIMUTH"])
    long_dist = track_msg["LONG_DIST"]
    return (
      math.cos(azimuth) * long_dist,
      -math.sin(azimuth) * long_dist,
      track_msg["REL_SPEED"],
    )

  if radar_spec.name == "RADAR_235_248":
    return (
      track_msg["LONG_DIST"],
      track_msg["LAT_DIST"],
      track_msg["REL_SPEED"],
    )

  return (
    track_msg["LONG_DIST"],
    track_msg["LAT_DIST"],
    track_msg["REL_SPEED"],
  )


def get_radar_track_state(radar_spec: HyundaiRadarTrackSpec, track_msg, track_prefix: str) -> int:
  state = int(track_msg[f"{track_prefix}STATE"])
  if radar_spec.name == "RADAR_210_21F" and state == 0:
    # Compact-dialect platforms leave detailed STATE zero and use the shared
    # compressed lifecycle field instead.
    return {0: 0, 1: 1, 2: 3, 3: 4}[int(track_msg[f"{track_prefix}STATE_ALT"])]
  return state


def get_radar_track_motion_state(radar_spec: HyundaiRadarTrackSpec, track_msg, track_prefix: str) -> int:
  motion_state = int(track_msg[f"{track_prefix}MOTION_STATE"])
  if radar_spec.name == "RADAR_235_248":
    # Normalize the camera-object enum to RadarPoint's common
    # 0=unknown, 1=stationary, 2=moving representation.
    if motion_state in (1, 2, 3, 6):
      return 1
    if motion_state in (5, 8, 9):
      return 2
    return 0
  return motion_state


def is_radar_track_valid(radar_spec: HyundaiRadarTrackSpec, track_msg, track_prefix: str) -> bool:
  if radar_spec.name == "RADAR_602_617":
    return 0 < track_msg[f"{track_prefix}DISTANCE"] < 255.75

  if radar_spec.name == "RADAR_235_248":
    return int(track_msg["OBJECT_ID"]) != 0

  return get_radar_track_state(radar_spec, track_msg, track_prefix) in (3, 4)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.pts: dict[int | HyundaiRadarTrackKey, structs.RadarData.RadarPoint] = {}
    detected_tracks = get_detected_radar_tracks(CP)
    self.radar_tracks_available = bool(detected_tracks)
    self.radar_tracks = detected_tracks
    self.radar_parsers = tuple(HyundaiRadarParserConfig(rt.spec, rt.bus, rt.end_addr, get_radar_can_parser(rt))
                               for rt in self.radar_tracks)
    self.seen_radar_addrs = {(rt.spec.name, rt.bus): set(rt.address_range) for rt in detected_tracks}
    self.updated_messages: set[tuple[int, int]] = set()
    self.active_radar_buses: dict[str, int] = {}
    self._locked_radar_buses: dict[str, int] = {}
    self._parser_by_bus_addr: dict[tuple[int, int], tuple[int, HyundaiRadarParserConfig]] = {}
    self._parser_track_counts: dict[tuple[str, int], int] = {}
    self._pending_completed_parsers: set[tuple[str, int]] = set()
    self._discovery_counter = 0
    self._last_publish_ns = 0
    self._is_full_radar = False
    self._is_lead_only = False
    self._rebuild_parser_lookup()
    self.trigger_msg = DEFAULT_RADAR_TRIGGER_MSG

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

  def _rebuild_parser_lookup(self) -> None:
    locked = self._locked_radar_buses
    self._parser_by_bus_addr = {
      (rp.bus, addr): (idx, rp)
      for idx, rp in enumerate(self.radar_parsers)
      if locked.get(rp.spec.name, rp.bus) == rp.bus
      for addr in rp.address_range
    }

  def _discover_radar_parsers(self, can_strings) -> None:
    existing_parsers = {(parser.spec.name, parser.bus): parser for parser in self.radar_parsers}
    updated_candidates: dict[tuple[str, int], HyundaiRadarTrackSpec] = {}
    lookup = self._parser_by_bus_addr
    for _, can_messages in can_strings:
      for addr, dat, bus in can_messages:
        if (bus, addr) in lookup:
          continue
        spec = RADAR_TRACK_SPEC_BY_ADDR.get(addr)
        if spec is None or bus not in RADAR_TRACK_BUSES or len(dat) != spec.message_size:
          continue

        parser_key = (spec.name, bus)
        self.seen_radar_addrs.setdefault(parser_key, set()).add(addr)
        if parser_key not in existing_parsers:
          updated_candidates[parser_key] = spec

    upgraded_tracks = {}
    for parser_key, parser in existing_parsers.items():
      if parser.end_addr == parser.spec.end_addr:
        continue
      seen_addrs = self.seen_radar_addrs[parser_key]
      if all(addr in seen_addrs for addr in parser.spec.address_range):
        upgraded_tracks[parser_key] = HyundaiRadarTrackConfig(parser.spec, parser.bus, parser.spec.end_addr)

    if upgraded_tracks:
      self.radar_tracks = tuple(
        upgraded_tracks.get((track.spec.name, track.bus), track)
        for track in self.radar_tracks
      )
      self.radar_parsers = tuple(
        HyundaiRadarParserConfig(track.spec, track.bus, track.end_addr, get_radar_can_parser(track))
        if (track.spec.name, track.bus) in upgraded_tracks else parser
        for track, parser in zip(self.radar_tracks, self.radar_parsers, strict=True)
      )
      self._rebuild_parser_lookup()
    discovered_tracks = []
    for (spec_name, bus), spec in updated_candidates.items():
      seen_addrs = self.seen_radar_addrs[(spec_name, bus)]
      if not all(addr in seen_addrs for addr in spec.required_address_range):
        continue
      end_addr = spec.end_addr if all(addr in seen_addrs for addr in spec.address_range) else spec.required_end_addr
      discovered_tracks.append(HyundaiRadarTrackConfig(spec, bus, end_addr))

    if not discovered_tracks:
      return

    self.radar_tracks = (*self.radar_tracks, *discovered_tracks)
    self.radar_parsers = (*self.radar_parsers, *(
      HyundaiRadarParserConfig(track.spec, track.bus, track.end_addr, get_radar_can_parser(track)) for track in discovered_tracks
    ))
    self._rebuild_parser_lookup()
    if self.rcp is None:
      self.rcp = self.radar_parsers[0].parser

  @property
  def radar_mode(self) -> int | None:
    return self._radar_mode

  def set_radar_mode(self, radar_mode: int) -> bool:
    if radar_mode not in (RadarType.OFF, RadarType.LEAD_ONLY, RadarType.FULL_RADAR):
      # A stale or hand-edited param must not take down card while driving.
      return False
    if radar_mode == self._radar_mode:
      return False

    radar_mode_flags = HyundaiFlagsSP.RADAR_LEAD_ONLY | HyundaiFlagsSP.RADAR_FULL_RADAR
    self.CP_SP.flags &= ~radar_mode_flags.value
    if radar_mode == RadarType.LEAD_ONLY:
      self.CP_SP.flags |= HyundaiFlagsSP.RADAR_LEAD_ONLY.value
    elif radar_mode == RadarType.FULL_RADAR:
      self.CP_SP.flags |= HyundaiFlagsSP.RADAR_FULL_RADAR.value

    self.pts.clear()
    self.updated_messages.clear()
    self.active_radar_buses.clear()
    self._locked_radar_buses.clear()
    self._parser_track_counts.clear()
    self._pending_completed_parsers.clear()
    self._last_publish_ns = 0
    self._is_full_radar = radar_mode == RadarType.FULL_RADAR
    self._is_lead_only = radar_mode == RadarType.LEAD_ONLY
    self._rebuild_parser_lookup()

    if radar_mode == RadarType.FULL_RADAR:
      self.trigger_msg = max((track.end_addr for track in self.radar_tracks), default=DEFAULT_RADAR_TRIGGER_MSG)
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

    if self._is_lead_only and self.use_radar_interface_ext:
      vls = self.rcp.update(can_strings)
      self.updated_messages.update((0, addr) for addr in vls)

      if (0, self.trigger_msg) not in self.updated_messages:
        return None

      rr = self._update(self.updated_messages)
      self.updated_messages.clear()
      return rr

    if not self.radar_parsers or self._discovery_counter % RADAR_DISCOVERY_INTERVAL == 0:
      self._discover_radar_parsers(can_strings)
    self._discovery_counter += 1

    num_parsers = len(self.radar_parsers)
    filtered_by_parser: list[list[tuple[int, list]]] = [[] for _ in range(num_parsers)]
    completed_by_parser = [False] * num_parsers
    cycle_ts_by_parser: list[list[int]] = [[] for _ in range(num_parsers)]
    lookup = self._parser_by_bus_addr
    latest_mono_time = 0

    for mono_time, can_messages in can_strings:
      if mono_time > latest_mono_time:
        latest_mono_time = mono_time
      batch_by_parser: dict[int, list] = {}
      for msg in can_messages:
        hit = lookup.get((msg[2], msg[0]))
        if hit is None:
          continue
        idx, rp = hit
        if len(msg[1]) != rp.spec.message_size:
          continue
        batch = batch_by_parser.get(idx)
        if batch is None:
          batch = []
          batch_by_parser[idx] = batch
        batch.append(msg)
        if msg[0] == rp.end_addr:
          completed_by_parser[idx] = True
          cycle_ts_by_parser[idx].append(mono_time)
      for idx, batch in batch_by_parser.items():
        filtered_by_parser[idx].append((mono_time, batch))

    for idx, radar_parser in enumerate(self.radar_parsers):
      filtered_can = filtered_by_parser[idx]
      if not filtered_can:
        continue

      # Feed every batch straight to the parser so track timestamps stay fresh and can_valid stays true.
      vls = radar_parser.parser.update(filtered_can)
      parser_key = (radar_parser.spec.name, radar_parser.bus)
      relevant_addrs = {
        addr for addr in vls
        if radar_parser.spec.start_addr <= addr <= radar_parser.end_addr
      }
      if relevant_addrs:
        self.seen_radar_addrs.setdefault(parser_key, set()).update(relevant_addrs)
        self.updated_messages.update((radar_parser.bus, addr) for addr in relevant_addrs)

      if not completed_by_parser[idx]:
        continue

      for timestamp in cycle_ts_by_parser[idx]:
        if not radar_parser.cycle_timestamps or timestamp != radar_parser.cycle_timestamps[-1]:
          radar_parser.cycle_timestamps.append(timestamp)
      self._pending_completed_parsers.add(parser_key)

    if not self._pending_completed_parsers:
      return None

    if self._last_publish_ns != 0 and latest_mono_time > self._last_publish_ns and \
       (latest_mono_time - self._last_publish_ns) < RADAR_PUBLISH_INTERVAL_NS:
      return None

    self._last_publish_ns = latest_mono_time
    rr = self._update(self.updated_messages, self._pending_completed_parsers)
    self.updated_messages.clear()
    self._pending_completed_parsers.clear()
    return rr

  def _radar_range_complete(self, radar_parser: HyundaiRadarParserConfig) -> bool:
    seen_addrs = self.seen_radar_addrs.get((radar_parser.spec.name, radar_parser.bus), set())
    return all(addr in seen_addrs for addr in radar_parser.address_range)

  @staticmethod
  def _radar_parser_valid_count(radar_parser: HyundaiRadarParserConfig, updated_messages: set[tuple[int, int]]) -> int:
    valid_count = 0
    spec = radar_parser.spec
    bus = radar_parser.bus
    for addr, track_prefix, _, ts_sig, msg, ts_msg in radar_parser.track_entries:
      if updated_messages and (bus, addr) not in updated_messages:
        continue
      if ts_msg[ts_sig] == 0:
        continue
      if is_radar_track_valid(spec, msg, track_prefix):
        valid_count += 1
    return valid_count

  @staticmethod
  def _radar_parser_cadence_score(radar_parser: HyundaiRadarParserConfig) -> int:
    timestamps = radar_parser.cycle_timestamps
    if len(timestamps) < 3:
      return 1

    elapsed = timestamps[-1] - timestamps[0]
    if elapsed <= 0:
      return 0

    observed_frequency = (len(timestamps) - 1) * 1e9 / elapsed
    relative_error = abs(observed_frequency - radar_parser.spec.frequency) / radar_parser.spec.frequency
    return 2 if relative_error <= RADAR_CADENCE_TOLERANCE else 0

  def _select_active_radar_parser(self, radar_parsers: list[HyundaiRadarParserConfig]) -> HyundaiRadarParserConfig:
    spec_name = radar_parsers[0].spec.name
    if len(radar_parsers) == 1:
      best_parser = radar_parsers[0]
      self.active_radar_buses[spec_name] = best_parser.bus
      return best_parser

    locked_bus = self._locked_radar_buses.get(spec_name)
    if locked_bus is not None:
      for rp in radar_parsers:
        if rp.bus == locked_bus:
          if rp.parser.can_valid:
            return rp
          # Active bus lost validity; unlock and re-enable standby parsers for failover
          del self._locked_radar_buses[spec_name]
          self._rebuild_parser_lookup()
          break

    valid_counts = {rp.bus: self._radar_parser_valid_count(rp, set()) for rp in radar_parsers}
    current_bus = self.active_radar_buses.get(spec_name)
    latest_cycle_timestamp = max((rp.cycle_timestamps[-1] for rp in radar_parsers if rp.cycle_timestamps), default=0)

    def parser_score(radar_parser: HyundaiRadarParserConfig) -> tuple[int, int, bool, int]:
      if latest_cycle_timestamp == 0:
        cycle_fresh = 1
      elif not radar_parser.cycle_timestamps:
        cycle_fresh = 0
      else:
        freshness_limit = RADAR_FRESH_CYCLE_PERIODS * 1e9 / radar_parser.spec.frequency
        cycle_fresh = int(latest_cycle_timestamp - radar_parser.cycle_timestamps[-1] <= freshness_limit)

      return (
        cycle_fresh,
        self._radar_parser_cadence_score(radar_parser),
        radar_parser.bus == current_bus,
        valid_counts[radar_parser.bus],
      )

    best_parser = max(radar_parsers, key=parser_score)

    self.active_radar_buses[spec_name] = best_parser.bus
    if len(best_parser.cycle_timestamps) >= 3 and best_parser.parser.can_valid:
      self._locked_radar_buses[spec_name] = best_parser.bus
      self._rebuild_parser_lookup()
    return best_parser

  def _update_radar_points_from_parser(self, radar_parser: HyundaiRadarParserConfig) -> int:
    track_count = 0
    spec = radar_parser.spec
    spec_name = spec.name
    bus = radar_parser.bus
    full_variant = radar_parser.end_addr == spec.end_addr
    has_track_metadata = spec_name in ("RADAR_210_21F", "RADAR_235_248", "RADAR_3A5_3C4")
    is_esr = spec_name == "RADAR_500_53F"
    pts = self.pts
    for addr, track_prefix, track_key, ts_sig, msg, ts_msg in radar_parser.track_entries:
      if ts_msg[ts_sig] != 0 and is_radar_track_valid(spec, msg, track_prefix):
        track_count += 1
        pt = pts.get(track_key)
        if pt is None:
          pt = structs.RadarData.RadarPoint()
          pt.trackId = self.track_id
          self.track_id += 1
          pts[track_key] = pt

        d_rel, y_rel, v_rel = decode_radar_track(spec, msg, track_prefix)
        pt.dRel = d_rel
        pt.yRel = y_rel
        pt.vRel = v_rel
        if is_esr:
          pt.motionState = 2 if full_variant and int(msg["ONCOMING_ESR"]) else 0
        else:
          pt.motionState = get_radar_track_motion_state(spec, msg, track_prefix) if has_track_metadata else 0
        pt.sourceAddress = addr
        pt.sourceBus = bus
        pt.trackAge = int(msg[f"{track_prefix}AGE"]) if has_track_metadata else min(pt.trackAge + 1, 65535)
      elif track_key in pts:
        del pts[track_key]
    return track_count

  def _update(self, updated_messages, completed_parsers: set[tuple[str, int]] | None = None):
    ret = structs.RadarData()
    ret.radarTracksAvailable = self.radar_tracks_available

    if self._is_lead_only:
      if self.use_radar_interface_ext and self.rcp is not None:
        return self.update_ext(ret)
      return ret

    active_radar_parsers: list[HyundaiRadarParserConfig] = []
    active_radar_sources: list[tuple[HyundaiRadarParserConfig, int]] = []
    if self._is_full_radar:
      radar_parsers_by_spec: dict[str, list[HyundaiRadarParserConfig]] = {}
      for radar_parser in self.radar_parsers:
        if not self._radar_range_complete(radar_parser):
          continue
        self.radar_tracks_available = True
        radar_parsers_by_spec.setdefault(radar_parser.spec.name, []).append(radar_parser)

      ret.radarTracksAvailable = self.radar_tracks_available

      for radar_parsers in radar_parsers_by_spec.values():
        prev_bus = self.active_radar_buses.get(radar_parsers[0].spec.name)
        active_radar_parser = self._select_active_radar_parser(radar_parsers)
        active_radar_parsers.append(active_radar_parser)
        parser_key = (active_radar_parser.spec.name, active_radar_parser.bus)
        if completed_parsers is None or parser_key in completed_parsers or active_radar_parser.bus != prev_bus:
          track_count = self._update_radar_points_from_parser(active_radar_parser)
          self._parser_track_counts[parser_key] = track_count
        else:
          track_count = self._parser_track_counts.get(parser_key, 0)
        active_radar_sources.append((active_radar_parser, track_count))

    if any(not radar_parser.parser.can_valid for radar_parser in active_radar_parsers):
      ret.errors.canError = True

    ret.trackSources = [{
      "startAddress": radar_parser.spec.start_addr,
      "endAddress": radar_parser.end_addr,
      "bus": radar_parser.bus,
      "trackCount": track_count,
    } for radar_parser, track_count in active_radar_sources]
    ret.points = list(self.pts.values())
    return ret
