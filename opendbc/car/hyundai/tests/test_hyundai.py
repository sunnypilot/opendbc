from hypothesis import settings, given, strategies as st

import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams, CarParamsSP
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.radar_interface import HYUNDAI_RADAR_TRACK_SPECS, RADAR_210_21F, RADAR_235_248, RADAR_3A5_3C4, \
                                                RADAR_500_51F, RADAR_602_617, RadarInterface, get_detected_radar_tracks, \
                                                is_radar_track_valid
from opendbc.car.hyundai.values import CAMERA_SCC_CAR, CANFD_CAR, CAN_GEARS, CAR, CHECKSUM, DATE_FW_ECUS, \
                                         HYBRID_CAR, EV_CAR, FW_QUERY_CONFIG, LEGACY_SAFETY_MODE_CAR, CANFD_FUZZY_WHITELIST, \
                                         UNSUPPORTED_LONGITUDINAL_CAR, PLATFORM_CODE_ECUS, HYUNDAI_VERSION_REQUEST_LONG, \
                                         HyundaiFlags, get_platform_codes, HyundaiSafetyFlags, \
                                         NON_SCC_CAR
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, RadarType

Ecu = CarParams.Ecu

# Some platforms have date codes in a different format we don't yet parse (or are missing).
# For now, assert list of expected missing date cars
NO_DATES_PLATFORMS = {
  # CAN FD
  CAR.KIA_SPORTAGE_5TH_GEN,
  CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN,
  CAR.HYUNDAI_SANTA_CRUZ_2025,
  CAR.HYUNDAI_TUCSON_4TH_GEN,
  CAR.HYUNDAI_TUCSON_2025,
  CAR.HYUNDAI_TUCSON_HEV_2025,
  CAR.HYUNDAI_TUCSON_PHEV_2025,
  # CAN
  CAR.HYUNDAI_ELANTRA,
  CAR.HYUNDAI_ELANTRA_GT_I30,
  CAR.KIA_CEED,
  CAR.KIA_FORTE,
  CAR.KIA_OPTIMA_G4,
  CAR.KIA_OPTIMA_G4_FL,
  CAR.KIA_SORENTO,
  CAR.HYUNDAI_KONA,
  CAR.HYUNDAI_KONA_EV,
  CAR.HYUNDAI_KONA_EV_2022,
  CAR.HYUNDAI_KONA_HEV,
  CAR.HYUNDAI_SONATA_LF,
  CAR.HYUNDAI_VELOSTER,
  CAR.HYUNDAI_KONA_2022,
}

CANFD_EXPECTED_ECUS = {Ecu.fwdCamera, Ecu.fwdRadar}


def add_radar_range(fingerprint, radar_spec, bus, missing_addr=None):
  for addr in range(radar_spec.start_addr, radar_spec.end_addr + 1):
    if addr != missing_addr:
      fingerprint[bus][addr] = radar_spec.message_size


def mark_track_updated(parser, radar_spec, addr, prefix=""):
  msg_name = f"RADAR_TRACK_{addr:x}"
  signal = "DISTANCE" if radar_spec == RADAR_602_617 else "LONG_DIST"
  parser.ts_nanos[msg_name][f"{prefix}{signal}"] = 1


def get_radar_parser(RI, radar_spec, bus):
  return next(rp for rp in RI.radar_parsers if rp.spec == radar_spec and rp.bus == bus)


class TestHyundaiFingerprint(unittest.TestCase):
  def test_radar_track_frequencies(self):
    assert {spec.name: spec.frequency for spec in HYUNDAI_RADAR_TRACK_SPECS} == {
      "RADAR_500_51F": 20,
      "RADAR_210_21F": 20,
      "RADAR_235_248": 33,
      "RADAR_3A5_3C4": 20,
      "RADAR_602_617": 10,
    }

  def test_radar_track_message_sizes(self):
    assert {spec.name: spec.message_size for spec in HYUNDAI_RADAR_TRACK_SPECS} == {
      "RADAR_500_51F": 8,
      "RADAR_210_21F": 32,
      "RADAR_235_248": 32,
      "RADAR_3A5_3C4": 24,
      "RADAR_602_617": 8,
    }

  def test_radar_602_track_validity(self):
    for distance, valid in ((0, False), (0.25, True), (255.5, True), (255.75, False)):
      with self.subTest(distance=distance):
        assert is_radar_track_valid(RADAR_602_617, {"1_DISTANCE": distance}, "1_") == valid

  def test_radar_602_second_track_speed(self):
    packer = CANPacker(RADAR_602_617.dbc_name)
    parser = CANParser(RADAR_602_617.dbc_name, [("RADAR_TRACK_602", RADAR_602_617.frequency)], 1)
    msg = packer.make_can_msg("RADAR_TRACK_602", 1, {"2_SPEED": -15})

    parser.update([(1, [msg])])

    assert parser.vl["RADAR_TRACK_602"]["2_SPEED"] == -15

  def test_radar_3a5_signal_layout(self):
    packer = CANPacker(RADAR_3A5_3C4.dbc_name)
    parser = CANParser(RADAR_3A5_3C4.dbc_name, [("RADAR_TRACK_3a5", RADAR_3A5_3C4.frequency)], 1)
    values = {
      "STATE_ALT": 3,
      "MOTION_STATE": 2,
      "TRACK_COUNTER": 3,
      "NEW_SIGNAL_2": -12,
      "AGE": 255,
      "COAST_AGE": 15,
      "STATE": 4,
      "NEW_SIGNAL_8": -21,
      "LONG_DIST": 409.55,
      "LAT_DIST": -61.55,
      "REL_SPEED": -66.79,
      "NEW_SIGNAL_4": 2,
      "REL_LAT_SPEED": -3.25,
      "REL_ACCEL": 10.22,
      "NEW_SIGNAL_18": 3,
      "NEW_SIGNAL_5": 32,
      "WIDTH": 3.1,
      "LENGTH": 15.5,
      "ABS_SPEED": 68.3,
      "ORIENTATION_ANGLE": -173,
      "NEW_SIGNAL_13": 10,
      "NEW_SIGNAL_12": 5,
      "NEW_SIGNAL_14": 1,
      "NEW_SIGNAL_15": 2,
      "NEW_SIGNAL_16": 3,
      "NEW_SIGNAL_17": 1,
    }

    parser.update([(1, [packer.make_can_msg("RADAR_TRACK_3a5", 1, values)])])
    parsed = parser.vl["RADAR_TRACK_3a5"]

    for signal, expected in values.items():
      assert abs(parsed[signal] - expected) < 1e-6
    assert "LAT_DIST_ACCEL" not in parsed

  def test_radar_3a5_known_ioniq9_frames(self):
    parser = CANParser(RADAR_3A5_3C4.dbc_name, [
      ("RADAR_TRACK_3af", RADAR_3A5_3C4.frequency),
      ("RADAR_TRACK_3c0", RADAR_3A5_3C4.frequency),
    ], 1)

    # The distance MSB is set, so this is 205.25 m rather than a wrapped 0.45 m.
    distance_frame = bytes.fromhex("d19ba383061f4d8004b8ee8303c7800000541f03cd271608")
    parser.update([(1, [(0x3AF, distance_frame, 1)])])
    assert parser.vl["RADAR_TRACK_3af"]["LONG_DIST"] == 205.25

    # The length MSB is set, so this is a 15.2 m long object rather than 2.4 m.
    dimensions_frame = bytes.fromhex("fb29791225ff3081a2400d313ffadffd016c98a728a0aa06")
    parser.update([(2, [(0x3C0, dimensions_frame, 1)])])
    dimensions = parser.vl["RADAR_TRACK_3c0"]
    assert dimensions["WIDTH"] == 2.7
    assert abs(dimensions["LENGTH"] - 15.2) < 1e-6
    assert dimensions["ORIENTATION_ANGLE"] == 10

  def test_feature_detection(self):
    # LKA steering
    for lka_steering in (True, False):
      fingerprint = gen_empty_fingerprint()
      if lka_steering:
        cam_can = CanBus(None, fingerprint).CAM
        fingerprint[cam_can] = [0x50, 0x110]  # LKA steering messages
      CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG) == lka_steering

    # radar available
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        add_radar_range(fingerprint, RADAR_500_51F, 1)
      CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_radar_track_detection(self):
    for radar_spec in HYUNDAI_RADAR_TRACK_SPECS:
      for bus in (0, 1, 2):
        with self.subTest(radar_spec=radar_spec.name, bus=bus):
          fingerprint = gen_empty_fingerprint()
          add_radar_range(fingerprint, radar_spec, bus)

          CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
          radar_tracks = get_detected_radar_tracks(CP)

          assert not CP.radarUnavailable
          assert len(radar_tracks) == 1
          assert radar_tracks[0].spec == radar_spec
          assert radar_tracks[0].bus == bus

  def test_radar_track_detection_requires_full_range(self):
    for radar_spec in HYUNDAI_RADAR_TRACK_SPECS:
      with self.subTest(radar_spec=radar_spec.name):
        fingerprint = gen_empty_fingerprint()
        add_radar_range(fingerprint, radar_spec, 1, missing_addr=radar_spec.end_addr)

        CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)

        assert CP.radarUnavailable
        assert get_detected_radar_tracks(CP) == ()

  def test_radar_track_detection_requires_correct_message_size(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_3A5_3C4, 0)
    add_radar_range(fingerprint, RADAR_3A5_3C4, 1)
    fingerprint[1][RADAR_3A5_3C4.start_addr] = 8

    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    radar_tracks = get_detected_radar_tracks(CP)

    assert [(rt.spec, rt.bus) for rt in radar_tracks] == [(RADAR_3A5_3C4, 0)]

  def test_radar_track_detection_multiple_ranges(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_500_51F, 1)
    add_radar_range(fingerprint, RADAR_3A5_3C4, 2)

    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    radar_tracks = get_detected_radar_tracks(CP)

    assert not CP.radarUnavailable
    assert [(rt.spec, rt.bus) for rt in radar_tracks] == [(RADAR_500_51F, 1), (RADAR_3A5_3C4, 2)]

  def test_radar_track_detection_keeps_duplicate_range_candidates(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_500_51F, 0)
    add_radar_range(fingerprint, RADAR_500_51F, 1)
    add_radar_range(fingerprint, RADAR_500_51F, 2)

    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    radar_tracks = get_detected_radar_tracks(CP)

    assert not CP.radarUnavailable
    assert [(rt.spec, rt.bus) for rt in radar_tracks] == [(RADAR_500_51F, 0), (RADAR_500_51F, 1), (RADAR_500_51F, 2)]

  def test_radar_interface_multiple_parsers(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_500_51F, 1)
    add_radar_range(fingerprint, RADAR_3A5_3C4, 2)
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)

    RI = RadarInterface(CP, CP_SP)

    radar_500_51f_parser = get_radar_parser(RI, RADAR_500_51F, 1)
    radar_3a5_3c4_parser = get_radar_parser(RI, RADAR_3A5_3C4, 2)
    radar_500_51f_msg = radar_500_51f_parser.parser.vl[f"RADAR_TRACK_{RADAR_500_51F.start_addr:x}"]
    radar_500_51f_msg["STATE"] = 3
    radar_500_51f_msg["LONG_DIST"] = 10
    radar_500_51f_msg["AZIMUTH"] = 0
    radar_500_51f_msg["REL_SPEED"] = 1
    radar_500_51f_msg["REL_ACCEL"] = 0
    mark_track_updated(radar_500_51f_parser.parser, RADAR_500_51F, RADAR_500_51F.start_addr)

    radar_3a5_3c4_msg = radar_3a5_3c4_parser.parser.vl[f"RADAR_TRACK_{RADAR_3A5_3C4.start_addr:x}"]
    radar_3a5_3c4_msg["STATE"] = 3
    radar_3a5_3c4_msg["MOTION_STATE"] = 2
    radar_3a5_3c4_msg["LONG_DIST"] = 20
    radar_3a5_3c4_msg["LAT_DIST"] = 1
    radar_3a5_3c4_msg["REL_SPEED"] = 2
    radar_3a5_3c4_msg["REL_ACCEL"] = 0
    mark_track_updated(radar_3a5_3c4_parser.parser, RADAR_3A5_3C4, RADAR_3A5_3C4.start_addr)

    radar_data = RI._update([])
    points = radar_data.points

    assert len(points) == 2
    assert [(source.startAddress, source.endAddress, source.bus, source.trackCount) for source in radar_data.trackSources] == [
      (RADAR_500_51F.start_addr, RADAR_500_51F.end_addr, 1, 1),
      (RADAR_3A5_3C4.start_addr, RADAR_3A5_3C4.end_addr, 2, 1),
    ]
    assert ("RADAR_500_51F", RADAR_500_51F.start_addr) in RI.pts
    assert ("RADAR_3A5_3C4", RADAR_3A5_3C4.start_addr) in RI.pts
    assert RI.pts[("RADAR_500_51F", RADAR_500_51F.start_addr)].motionState == 0
    assert RI.pts[("RADAR_3A5_3C4", RADAR_3A5_3C4.start_addr)].motionState == 2
    assert RI.pts[("RADAR_500_51F", RADAR_500_51F.start_addr)].sourceAddress == RADAR_500_51F.start_addr
    assert RI.pts[("RADAR_3A5_3C4", RADAR_3A5_3C4.start_addr)].sourceAddress == RADAR_3A5_3C4.start_addr
    assert RI.pts[("RADAR_500_51F", RADAR_500_51F.start_addr)].sourceBus == 1
    assert RI.pts[("RADAR_3A5_3C4", RADAR_3A5_3C4.start_addr)].sourceBus == 2
    assert RI.pts[("RADAR_500_51F", RADAR_500_51F.start_addr)].trackAge == 1
    assert RI.pts[("RADAR_3A5_3C4", RADAR_3A5_3C4.start_addr)].trackAge == 1

  def test_radar_interface_duplicate_range_selects_populated_bus(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_210_21F, 0)
    add_radar_range(fingerprint, RADAR_210_21F, 2)
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)

    RI = RadarInterface(CP, CP_SP)

    bus0_parser = get_radar_parser(RI, RADAR_210_21F, 0).parser
    bus2_parser = get_radar_parser(RI, RADAR_210_21F, 2).parser
    msg = bus2_parser.vl[f"RADAR_TRACK_{RADAR_210_21F.start_addr:x}"]
    msg["1_STATE"] = 3
    msg["1_LONG_DIST"] = 10
    msg["1_LAT_DIST"] = 1
    msg["1_REL_SPEED"] = 2
    mark_track_updated(bus2_parser, RADAR_210_21F, RADAR_210_21F.start_addr, "1_")

    mark_track_updated(bus0_parser, RADAR_210_21F, RADAR_210_21F.start_addr, "1_")

    points = RI._update({(0, RADAR_210_21F.start_addr), (2, RADAR_210_21F.start_addr)}).points

    assert len(points) == 1
    assert RI.active_radar_buses[RADAR_210_21F.name] == 2

  def test_radar_interface_duplicate_range_selects_correct_cadence(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_3A5_3C4, 0)
    add_radar_range(fingerprint, RADAR_3A5_3C4, 2)
    CP = CarInterface.get_params(CAR.KIA_SPORTAGE_5TH_GEN, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)
    RI = RadarInterface(CP, CP_SP)

    bus0_parser = get_radar_parser(RI, RADAR_3A5_3C4, 0)
    bus2_parser = get_radar_parser(RI, RADAR_3A5_3C4, 2)
    bus0_parser.cycle_timestamps.extend(range(0, 500_000_000, 50_000_000))
    bus2_parser.cycle_timestamps.extend(range(0, 500_000_000, 100_000_000))

    RI._update(set())

    assert RI.active_radar_buses[RADAR_3A5_3C4.name] == 0

  def test_radar_interface_duplicate_range_rejects_stale_echo(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_3A5_3C4, 0)
    add_radar_range(fingerprint, RADAR_3A5_3C4, 2)
    CP = CarInterface.get_params(CAR.KIA_SPORTAGE_5TH_GEN, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)
    RI = RadarInterface(CP, CP_SP)

    bus0_parser = get_radar_parser(RI, RADAR_3A5_3C4, 0)
    bus2_parser = get_radar_parser(RI, RADAR_3A5_3C4, 2)
    bus0_parser.cycle_timestamps.extend(range(500_000_000, 1_000_000_000, 50_000_000))
    bus2_parser.cycle_timestamps.extend(range(0, 500_000_000, 50_000_000))

    RI.active_radar_buses[RADAR_3A5_3C4.name] = 2
    RI._update(set())

    assert RI.active_radar_buses[RADAR_3A5_3C4.name] == 0

  def test_radar_interface_detects_late_live_range(self):
    fingerprint = gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)

    RI = RadarInterface(CP, CP_SP)
    radar_messages = [(addr, b'\x00' * RADAR_235_248.message_size, 1) for addr in RADAR_235_248.address_range]
    RI._discover_radar_parsers([(0, radar_messages)])
    radar_parser = get_radar_parser(RI, RADAR_235_248, 1)
    assert len(RI.radar_parsers) == 1
    assert len(RI._update([]).points) == 0

    for addr in range(RADAR_235_248.start_addr, RADAR_235_248.end_addr + 1):
      RI.seen_radar_addrs.setdefault((RADAR_235_248.name, 1), set()).add(addr)

    msg = radar_parser.parser.vl[f"RADAR_TRACK_{RADAR_235_248.start_addr:x}"]
    msg["STATE"] = 3
    msg["LONG_DIST"] = 10
    msg["LAT_DIST"] = 1
    msg["REL_SPEED"] = 2
    mark_track_updated(radar_parser.parser, RADAR_235_248, RADAR_235_248.start_addr)

    points = RI._update({(1, RADAR_235_248.start_addr)}).points

    assert len(points) == 1
    assert RI.active_radar_buses[RADAR_235_248.name] == 1

  def test_radar_interface_live_detection_requires_correct_message_size(self):
    fingerprint = gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.KIA_SPORTAGE_5TH_GEN, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)
    RI = RadarInterface(CP, CP_SP)

    wrong_size_messages = [(addr, b'\x00' * 8, 1) for addr in RADAR_3A5_3C4.address_range]
    RI._discover_radar_parsers([(0, wrong_size_messages)])
    assert len(RI.radar_parsers) == 0

    radar_messages = [(addr, b'\x00' * RADAR_3A5_3C4.message_size, 0) for addr in RADAR_3A5_3C4.address_range]
    RI._discover_radar_parsers([(1, radar_messages)])

    assert [(rp.spec, rp.bus) for rp in RI.radar_parsers] == [(RADAR_3A5_3C4, 0)]

  def test_radar_interface_publishes_on_complete_cycle(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_235_248, 1)
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)
    RI = RadarInterface(CP, CP_SP)

    dat = b'\x00' * RADAR_235_248.message_size
    assert RI.update([(123, [(RADAR_235_248.start_addr, dat, 1)])]) is None
    assert len(RI.pending_radar_can[(RADAR_235_248.name, 1)]) == 1

    radar_data = RI.update([(124, [(RADAR_235_248.end_addr, dat, 1)])])
    assert radar_data is not None
    assert RI.updated_messages == set()
    assert RI.pending_radar_can == {}

  def test_radar_interface_discards_incomplete_cycle_on_next_start(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_235_248, 1)
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)
    RI = RadarInterface(CP, CP_SP)
    parser_key = (RADAR_235_248.name, 1)

    dat = b'\x00' * RADAR_235_248.message_size
    assert RI.update([(123, [(RADAR_235_248.start_addr + 1, dat, 1)])]) is None
    assert RI.update([(124, [(RADAR_235_248.start_addr, dat, 1)])]) is None
    assert RI.pending_radar_can[parser_key] == [(124, [(RADAR_235_248.start_addr, dat, 1)])]

  def test_radar_interface_bounds_incomplete_cycle_storage(self):
    fingerprint = gen_empty_fingerprint()
    add_radar_range(fingerprint, RADAR_235_248, 1)
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)
    RI = RadarInterface(CP, CP_SP)
    parser_key = (RADAR_235_248.name, 1)
    pending_limit = RADAR_235_248.msg_count * 2

    for mono_time in range(pending_limit + 5):
      dat = b'\x00' * RADAR_235_248.message_size
      assert RI.update([(mono_time, [(RADAR_235_248.start_addr + 1, dat, 1)])]) is None

    assert len(RI.pending_radar_can[parser_key]) == pending_limit
    assert RI.pending_radar_can[parser_key][0][0] == 5

  def test_radar_interface_live_mode_switch(self):
    fingerprint = gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    CP_SP = CarParamsSP()
    RI = RadarInterface(CP, CP_SP)

    assert RI.radar_off_can
    assert RI.rcp is None
    assert not RI._update(set()).radarTracksAvailable

    assert RI.set_radar_mode(RadarType.FULL_RADAR)
    assert CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR
    assert not CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY
    assert not RI.radar_off_can

    radar_messages = [(addr, b'\x00' * RADAR_235_248.message_size, 1) for addr in RADAR_235_248.address_range]
    RI._discover_radar_parsers([(0, radar_messages)])
    radar_parser = get_radar_parser(RI, RADAR_235_248, 1)
    for addr in RADAR_235_248.address_range:
      RI.seen_radar_addrs.setdefault((RADAR_235_248.name, 1), set()).add(addr)

    msg = radar_parser.parser.vl[f"RADAR_TRACK_{RADAR_235_248.start_addr:x}"]
    msg["STATE"] = 3
    msg["LONG_DIST"] = 10
    msg["LAT_DIST"] = 1
    msg["REL_SPEED"] = 2
    mark_track_updated(radar_parser.parser, RADAR_235_248, RADAR_235_248.start_addr)

    radar_data = RI._update({(1, RADAR_235_248.start_addr)})
    assert len(radar_data.points) == 1
    assert radar_data.radarTracksAvailable

    assert RI.set_radar_mode(RadarType.OFF)
    assert not CP_SP.flags & (HyundaiFlagsSP.RADAR_LEAD_ONLY | HyundaiFlagsSP.RADAR_FULL_RADAR)
    assert RI.radar_off_can
    assert RI.rcp is None
    assert len(RI.pts) == 0
    radar_data = RI._update(set())
    assert len(radar_data.points) == 0
    assert radar_data.radarTracksAvailable

    assert RI.set_radar_mode(RadarType.FULL_RADAR)
    assert not RI.radar_off_can
    assert not RI.set_radar_mode(RadarType.FULL_RADAR)

  def test_radar_interface_two_track_ranges(self):
    for radar_spec in (RADAR_210_21F, RADAR_602_617):
      with self.subTest(radar_spec=radar_spec.name):
        fingerprint = gen_empty_fingerprint()
        add_radar_range(fingerprint, radar_spec, 1)
        CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
        CP_SP = CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)
        RI = RadarInterface(CP, CP_SP)

        parser = get_radar_parser(RI, radar_spec, 1).parser
        msg = parser.vl[f"RADAR_TRACK_{radar_spec.start_addr:x}"]
        if radar_spec == RADAR_210_21F:
          for i in ("1_", "2_"):
            msg[f"{i}STATE"] = 3
            msg[f"{i}LONG_DIST"] = 10
            msg[f"{i}LAT_DIST"] = 1
            msg[f"{i}REL_SPEED"] = 2
            mark_track_updated(parser, radar_spec, radar_spec.start_addr, i)
        else:
          for addr in range(radar_spec.start_addr, radar_spec.end_addr + 1):
            for i in ("1_", "2_"):
              parser.vl[f"RADAR_TRACK_{addr:x}"][f"{i}DISTANCE"] = 255.75
          for i in ("1_", "2_"):
            msg[f"{i}DISTANCE"] = 10
            msg[f"{i}LATERAL"] = 1
            msg[f"{i}SPEED"] = 2
            mark_track_updated(parser, radar_spec, radar_spec.start_addr, i)

        points = RI._update([]).points

        assert len(points) == 2

  def test_alternate_limits(self):
    # Alternate lateral control limits, for high torque cars, verify Panda safety mode flag is set
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.ALT_LIMITS) == bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.ALT_LIMITS)

  def test_can_features(self):
    # Test no EV/HEV in any gear lists (should all use ELECT_GEAR)
    assert set.union(*CAN_GEARS.values()) & (HYBRID_CAR | EV_CAR) == set()

    # Test CAN FD car not in CAN feature lists
    can_specific_feature_list = set.union(*CAN_GEARS.values(), *CHECKSUM.values(), LEGACY_SAFETY_MODE_CAR,
                                          *UNSUPPORTED_LONGITUDINAL_CAR.values(), CAMERA_SCC_CAR)
    for car_model in CANFD_CAR:
      assert car_model not in can_specific_feature_list, "CAN FD car unexpectedly found in a CAN feature list"

  def test_hybrid_ev_sets(self):
    assert HYBRID_CAR & EV_CAR == set(), "Shared cars between hybrid and EV"
    assert CANFD_CAR & HYBRID_CAR == set(), "Hard coding CAN FD cars as hybrid is no longer supported"

  def test_canfd_ecu_whitelist(self):
    # Asserts only expected Ecus can exist in database for CAN-FD cars
    for car_model in CANFD_CAR:
      ecus = {fw[0] for fw in FW_VERSIONS[car_model].keys()}
      ecus_not_in_whitelist = ecus - CANFD_EXPECTED_ECUS
      ecu_strings = ", ".join([f"Ecu.{ecu}" for ecu in ecus_not_in_whitelist])
      assert len(ecus_not_in_whitelist) == 0, \
                       f"{car_model}: Car model has unexpected ECUs: {ecu_strings}"

  def test_blacklisted_parts(self):
    # Asserts no ECUs known to be shared across platforms exist in the database.
    # Tucson having Santa Cruz camera and EPS for example
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        if car_model == CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN:
          raise unittest.SkipTest("Skip checking Santa Cruz for its parts")

        for code, _ in get_platform_codes(ecus[(Ecu.fwdCamera, 0x7c4, None)]):
          if b"-" not in code:
            continue
          part = code.split(b"-")[1]
          assert not part.startswith(b'CW'), "Car has bad part number"

  def test_correct_ecu_response_database(self):
    """
    Assert standard responses for certain ECUs, since they can
    respond to multiple queries with different data
    """
    expected_fw_prefix = HYUNDAI_VERSION_REQUEST_LONG[1:]
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          assert all(fw.startswith(expected_fw_prefix) for fw in fws), \
                          f"FW from unexpected request in database: {(ecu, fws)}"

  @settings(max_examples=100)
  @given(data=st.data())
  def test_platform_codes_fuzzy_fw(self, data):
    """Ensure function doesn't raise an exception"""
    fw_strategy = st.lists(st.binary())
    fws = data.draw(fw_strategy)
    get_platform_codes(fws)

  def test_expected_platform_codes(self):
    # Ensures we don't accidentally add multiple platform codes for a car unless it is intentional
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          # Third and fourth character are usually EV/hybrid identifiers
          codes = {code.split(b"-")[0][:2] for code, _ in get_platform_codes(fws)}
          if car_model == CAR.HYUNDAI_PALISADE:
            assert codes == {b"LX", b"ON"}, f"Car has unexpected platform codes: {car_model} {codes}"
          elif car_model == CAR.HYUNDAI_KONA_EV and ecu[0] == Ecu.fwdCamera:
            assert codes == {b"OE", b"OS"}, f"Car has unexpected platform codes: {car_model} {codes}"
          else:
            assert len(codes) == 1, f"Car has multiple platform codes: {car_model} {codes}"

  # Tests for platform codes, part numbers, and FW dates which Hyundai will use to fuzzy
  # fingerprint in the absence of full FW matches:
  def test_platform_code_ecus_available(self):
    # TODO: add queries for these non-CAN FD cars to get EPS
    no_eps_platforms = CANFD_CAR | {CAR.KIA_SORENTO, CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H, CAR.KIA_K7_2017,
                                    CAR.KIA_OPTIMA_H_G4_FL, CAR.HYUNDAI_SONATA_LF, CAR.HYUNDAI_TUCSON, CAR.GENESIS_G90, CAR.GENESIS_G80, CAR.HYUNDAI_ELANTRA}

    # Asserts ECU keys essential for fuzzy fingerprinting are available on all platforms
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for platform_code_ecu in PLATFORM_CODE_ECUS:
          if platform_code_ecu in (Ecu.fwdRadar, Ecu.eps) and car_model == CAR.HYUNDAI_GENESIS:
            continue
          if platform_code_ecu == Ecu.eps and car_model in no_eps_platforms:
            continue
          if car_model in NON_SCC_CAR:
            continue
          assert platform_code_ecu in [e[0] for e in ecus]

  def test_fw_format(self):
    # Asserts:
    # - every supported ECU FW version returns one platform code
    # - every supported ECU FW version has a part number
    # - expected parsing of ECU FW dates

    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          if car_model in NON_SCC_CAR:
            continue

          codes = set()
          for fw in fws:
            result = get_platform_codes([fw])
            assert 1 == len(result), f"Unable to parse FW: {fw}"
            codes |= result

          if ecu[0] not in DATE_FW_ECUS or car_model in NO_DATES_PLATFORMS:
            assert all(date is None for _, date in codes)
          else:
            assert all(date is not None for _, date in codes)

          if car_model == CAR.HYUNDAI_GENESIS:
            raise unittest.SkipTest("No part numbers for car model")

          # Hyundai places the ECU part number in their FW versions, assert all parsable
          # Some examples of valid formats: b"56310-L0010", b"56310L0010", b"56310/M6300"
          assert all(b"-" in code for code, _ in codes), \
                          f"FW does not have part number: {fw}"

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([b"\xf1\x00DH LKAS 1.1 -150210"])
    assert results == {(b"DH", b"150210")}

    # Some cameras and all radars do not have dates
    results = get_platform_codes([b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         "])
    assert results == {(b"AEhe-G2000", None)}

    results = get_platform_codes([b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         "])
    assert results == {(b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00DH LKAS 1.1 -150210",
      b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         ",
      b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         ",
    ])
    assert results == {(b"DH", b"150210"), (b"AEhe-G2000", None), (b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.07 99211-S8100 220222",
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.08 99211-S8100 211103",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.01 99211-S9100 190405",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.03 99211-S9100 190720",
    ])
    assert results == {(b"LX2-S8100", b"220222"), (b"LX2-S8100", b"211103"),
                               (b"ON-S9100", b"190405"), (b"ON-S9100", b"190720")}

  def test_fuzzy_excluded_platforms(self):
    # Asserts a list of platforms that will not fuzzy fingerprint with platform codes due to them being shared.
    # This list can be shrunk as we combine platforms and detect features
    excluded_platforms = {
      CAR.GENESIS_G70,            # shared platform code, part number, and date
      CAR.GENESIS_G70_2020,
    }
    excluded_platforms |= CANFD_CAR - EV_CAR - CANFD_FUZZY_WHITELIST  # shared platform codes
    excluded_platforms |= NO_DATES_PLATFORMS  # date codes are required to match

    platforms_with_shared_codes = set()
    for platform, fw_by_addr in FW_VERSIONS.items():
      car_fw = []
      for ecu, fw_versions in fw_by_addr.items():
        ecu_name, addr, sub_addr = ecu
        for fw in fw_versions:
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

      if platform in NON_SCC_CAR:
        continue

      CP = CarParams(carFw=car_fw)
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
      if len(matches) == 1:
        assert list(matches)[0] == platform
      else:
        platforms_with_shared_codes.add(platform)

    assert platforms_with_shared_codes == excluded_platforms
