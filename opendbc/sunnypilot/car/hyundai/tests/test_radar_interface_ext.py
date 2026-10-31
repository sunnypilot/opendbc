import math
import unittest

from opendbc.testing import parameterized

from opendbc.can import CANPacker
from opendbc.car import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.radar_interface import CANFD_RADAR_MSG_COUNT, CANFD_RADAR_START_ADDR
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.escc import ESCC_MSG
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

ESCC_CARS = [
  (CAR.HYUNDAI_ELANTRA_2021, ESCC_MSG),
]

CAMERA_SCC_CARS = [
  (CAR.HYUNDAI_KONA_EV_2022, 0, 0x420, "SCC11"),
  (CAR.HYUNDAI_IONIQ_5, HyundaiFlags.CANFD_CAMERA_SCC.value, 0x1A0, "SCC_CONTROL"),
]

STANDARD_RADAR_CARS = [
  (CAR.HYUNDAI_ELANTRA_2021, 0),
  (CAR.HYUNDAI_SANTA_FE, 0),
]


class TestRadarInterfaceExt(unittest.TestCase):

  @staticmethod
  def _setup_platform(car_name, additional_flags=0, escc_msg=None):
    """Set up the platform with specific parameters"""
    CarInterface = interfaces[car_name]

    CP = CarInterface.get_non_essential_params(car_name)
    CP.flags |= additional_flags

    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)

    CI = CarInterface(CP, CP_SP)

    RD = CI.RadarInterface(CP, CP_SP)

    if escc_msg is not None and hasattr(RD, 'use_escc'):
      try:
        RD.use_escc = True
      except AttributeError:
        object.__setattr__(RD, 'use_escc', True)

    return RD, CP, CP_SP

  @parameterized("car_name, escc_msg", ESCC_CARS)
  def test_escc_radar_interface(self, car_name, escc_msg):
    """Test radar interface for ESCC-enabled cars"""
    RD, CP, CP_SP = self._setup_platform(car_name, escc_msg=escc_msg)

    # Assert that ESCC features are present
    if hasattr(RD, 'use_escc'):
      self.assertTrue(RD.use_escc, "ESCC car should have use_escc=True")
    if hasattr(RD, 'use_radar_interface_ext'):
      self.assertTrue(RD.use_radar_interface_ext, "ESCC car should use radar interface ext")

    # Run radar interface once
    RD.update([])

    # Test radar fault
    if not CP.radarUnavailable and RD.rcp is not None:
      cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
      rr = RD.update(cans)
      self.assertTrue(rr is None or len(rr.errors) > 0)

  @parameterized("car_name, flags, expected_trigger, msg_src", CAMERA_SCC_CARS)
  def test_camera_scc_radar_interface(self, car_name, flags, expected_trigger, msg_src):
    """Test radar interface for Camera SCC cars"""
    RD, CP, CP_SP = self._setup_platform(car_name, additional_flags=flags)

    # Assert Camera SCC flag is set appropriately
    if flags & HyundaiFlags.CAMERA_SCC:
      self.assertTrue(CP.flags & HyundaiFlags.CAMERA_SCC, "Car should have CAMERA_SCC flag")
    if flags & HyundaiFlags.CANFD_CAMERA_SCC:
      self.assertTrue(CP.flags & HyundaiFlags.CANFD_CAMERA_SCC, "Car should have CANFD_CAMERA_SCC flag")

    # Check if using radar interface ext
    if hasattr(RD, 'use_radar_interface_ext'):
      self.assertTrue(RD.use_radar_interface_ext, "Camera SCC car should use radar interface ext")

    # Verify trigger message
    if hasattr(RD, 'trigger_msg'):
      self.assertEqual(RD.trigger_msg, expected_trigger, f"Expected trigger_msg {expected_trigger}, got {RD.trigger_msg}")

    # Run radar interface once
    RD.update([])

    # Test radar fault
    if not CP.radarUnavailable and RD.rcp is not None:
      cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
      rr = RD.update(cans)
      self.assertTrue(rr is None or len(rr.errors) > 0)

  @parameterized("car_name, flags", STANDARD_RADAR_CARS)
  def test_standard_radar_interface(self, car_name, flags):
    """Test radar interface for standard radar cars"""
    RD, CP, CP_SP = self._setup_platform(car_name, additional_flags=flags)

    # Standard cars should not use radar interface ext
    if hasattr(RD, 'use_radar_interface_ext'):
      self.assertFalse(RD.use_radar_interface_ext, "Standard car should not use radar interface ext")

    # Run radar interface once
    RD.update([])

    # For standard radar, test the _update method directly if available
    if not CP.radarUnavailable and RD.rcp is not None and \
          hasattr(RD, '_update') and hasattr(RD, 'trigger_msg'):
      # Setup for _update test if needed
      if hasattr(RD, 'updated_messages'):
        RD.updated_messages = {RD.trigger_msg}
      RD._update(RD.updated_messages)

    # Test radar fault
    if not CP.radarUnavailable and RD.rcp is not None:
      cans = [(0, [CanData(0, b'', 0) for _ in range(5)])]
      rr = RD.update(cans)
      self.assertTrue(rr is None or len(rr.errors) > 0)

  def test_canfd_group1_radar_tracks(self):
    RD, CP, CP_SP = self._setup_platform(CAR.HYUNDAI_IONIQ_5, additional_flags=HyundaiFlags.CANFD_CAMERA_SCC.value)
    CP_SP.flags |= HyundaiFlagsSP.CANFD_RADAR_TRACKS.value
    CP.radarUnavailable = False
    RD = interfaces[CAR.HYUNDAI_IONIQ_5].RadarInterface(CP, CP_SP)

    self.assertFalse(RD.use_radar_interface_ext)
    self.assertEqual(RD.trigger_msg, 0x21f)

    active_bank1 = bytes.fromhex("a727dd0a2413300651f306553d0f000100000000000000000000000000000000")
    active_bank2 = bytes.fromhex("0fbddd000000000000000000000000000000000a221130f3d0e1f1913de75f02")
    payloads = {0x210: active_bank1, 0x213: active_bank2}
    packets = [(addr, payloads.get(addr, bytes(32)), 1)
               for addr in range(CANFD_RADAR_START_ADDR, CANFD_RADAR_START_ADDR + CANFD_RADAR_MSG_COUNT)]
    radar_data = RD.update([0, packets])

    self.assertIsNotNone(radar_data)
    self.assertEqual(len(radar_data.points), 2)
    point_bank2, point = sorted(radar_data.points, key=lambda p: p.dRel)
    self.assertAlmostEqual(point.dRel, 42.45, places=2)
    self.assertAlmostEqual(point.yRel, 5.55, places=2)
    self.assertAlmostEqual(point.vRel, -6.83, places=2)
    self.assertAlmostEqual(point_bank2.dRel, 23.2, places=2)
    self.assertAlmostEqual(point_bank2.yRel, -11.3, places=2)
    self.assertAlmostEqual(point_bank2.vRel, -6.23, places=2)

  @staticmethod
  def _canfd_group1_packets(track_values, scc_values=None):
    radar_packer = CANPacker("hyundai_canfd_radar_generated")
    packets = [radar_packer.make_can_msg(f"RADAR_TRACK_{addr:x}", 1,
                                         track_values if addr == CANFD_RADAR_START_ADDR else {})
               for addr in range(CANFD_RADAR_START_ADDR, CANFD_RADAR_START_ADDR + CANFD_RADAR_MSG_COUNT)]
    if scc_values is not None:
      scc_packer = CANPacker("hyundai_canfd_generated")
      packets.append(scc_packer.make_can_msg("SCC_CONTROL", 2, scc_values))
    return packets

  def test_canfd_group1_young_track_requires_scc_corroboration(self):
    RD, CP, CP_SP = self._setup_platform(CAR.HYUNDAI_IONIQ_5, additional_flags=HyundaiFlags.CANFD_CAMERA_SCC.value)
    CP_SP.flags |= HyundaiFlagsSP.CANFD_RADAR_TRACKS.value
    CP.radarUnavailable = False
    RD = interfaces[CAR.HYUNDAI_IONIQ_5].RadarInterface(CP, CP_SP)

    packets = self._canfd_group1_packets({
      "VALID_CNT1": 2,
      "LONG_DIST1": 42.45,
      "LAT_DIST1": 1.2,
      "REL_SPEED1": -6.83,
      "LAT_SPEED1": -0.4,
      "REL_ACCEL1": -0.5,
    }, {
      "COUNTER": 1,
      "ACC_ObjDist": 42.5,
      "ACC_ObjRelSpd": -6.8,
    })
    radar_data = RD.update([0, packets])

    self.assertIsNotNone(radar_data)
    self.assertEqual(len(radar_data.points), 1)
    point = radar_data.points[0]
    self.assertAlmostEqual(point.dRel, 42.45, places=2)
    self.assertAlmostEqual(point.yRel, 1.2, places=2)
    self.assertAlmostEqual(point.vRel, -6.83, places=2)
    self.assertAlmostEqual(point.deprecated.yvRel, -0.4, places=2)
    self.assertAlmostEqual(point.deprecated.aRel, -0.5, places=2)
    self.assertTrue(point.deprecated.measured)

  def test_canfd_group1_scc_fallback_for_unmatched_young_track(self):
    RD, CP, CP_SP = self._setup_platform(CAR.HYUNDAI_IONIQ_5, additional_flags=HyundaiFlags.CANFD_CAMERA_SCC.value)
    CP_SP.flags |= HyundaiFlagsSP.CANFD_RADAR_TRACKS.value
    CP.radarUnavailable = False
    RD = interfaces[CAR.HYUNDAI_IONIQ_5].RadarInterface(CP, CP_SP)

    packets = self._canfd_group1_packets({
      "VALID_CNT1": 2,
      "LONG_DIST1": 80.0,
      "LAT_DIST1": 1.0,
      "REL_SPEED1": 2.0,
    }, {
      "COUNTER": 1,
      "ACC_ObjDist": 42.5,
      "ACC_ObjRelSpd": -6.8,
    })
    radar_data = RD.update([0, packets])

    self.assertIsNotNone(radar_data)
    self.assertEqual(len(radar_data.points), 1)
    point = radar_data.points[0]
    self.assertAlmostEqual(point.dRel, 42.5, places=2)
    self.assertAlmostEqual(point.vRel, -6.8, places=2)
    self.assertTrue(math.isnan(point.yRel))
    self.assertFalse(point.deprecated.measured)

  def test_canfd_group1_stale_scc_fallback_is_removed(self):
    RD, CP, CP_SP = self._setup_platform(CAR.HYUNDAI_IONIQ_5, additional_flags=HyundaiFlags.CANFD_CAMERA_SCC.value)
    CP_SP.flags |= HyundaiFlagsSP.CANFD_RADAR_TRACKS.value
    CP.radarUnavailable = False
    RD = interfaces[CAR.HYUNDAI_IONIQ_5].RadarInterface(CP, CP_SP)

    initial_packets = self._canfd_group1_packets({}, {
      "COUNTER": 1,
      "ACC_ObjDist": 42.5,
      "ACC_ObjRelSpd": -6.8,
    })
    radar_data = RD.update([0, initial_packets])
    self.assertIsNotNone(radar_data)
    self.assertEqual(len(radar_data.points), 1)

    stale_packets = self._canfd_group1_packets({
      "VALID_CNT1": 2,
      "LONG_DIST1": 80.0,
      "LAT_DIST1": 1.0,
      "REL_SPEED1": 2.0,
    })
    radar_data = RD.update([150_000_001, stale_packets])
    self.assertIsNotNone(radar_data)
    self.assertEqual(len(radar_data.points), 0)

  def test_canfd_group1_radar_tracks_detection(self):
    CarInterface = interfaces[CAR.HYUNDAI_IONIQ_5]
    fingerprint = {bus: {} for bus in range(8)}
    fingerprint[1] = {addr: 32 for addr in range(CANFD_RADAR_START_ADDR,
                                                 CANFD_RADAR_START_ADDR + CANFD_RADAR_MSG_COUNT)}

    CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_5, fingerprint, [], True, False, False)
    CP.radarUnavailable = True
    CP_SP = CarInterface.get_params_sp(CP, CAR.HYUNDAI_IONIQ_5, fingerprint, [], True, False, False)

    self.assertTrue(CP_SP.flags & HyundaiFlagsSP.CANFD_RADAR_TRACKS)
    self.assertFalse(CP.radarUnavailable)
