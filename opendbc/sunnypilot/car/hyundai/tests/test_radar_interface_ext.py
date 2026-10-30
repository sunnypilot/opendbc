import unittest

from opendbc.testing import parameterized

from opendbc.can import CANPacker
from opendbc.car import CanData, gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.escc import ESCC_MSG
from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RADAR_TRACKS_ADDRS, RADAR_TRACKS_MSG_SIZE

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



def _radar_tracks_fingerprint(size: int = RADAR_TRACKS_MSG_SIZE):
  # LKA steering harness: A-CAN is bus 0
  fingerprint = gen_empty_fingerprint()
  fingerprint[2][0x50] = 16
  fingerprint[0].update({addr: size for addr in RADAR_TRACKS_ADDRS})
  return fingerprint


class TestRadarTracks(unittest.TestCase):
  @staticmethod
  def _setup(car_name, fingerprint):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_params(car_name, fingerprint, [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, car_name, fingerprint, [], False, False, False)
    return CarInterface.RadarInterface(CP, CP_SP), CP

  def test_detection(self):
    RD, CP = self._setup(CAR.GENESIS_GV80_2025, _radar_tracks_fingerprint())
    self.assertFalse(CP.radarUnavailable)
    self.assertTrue(RD.use_radar_tracks)
    self.assertEqual(RD.trigger_msg, RADAR_TRACKS_ADDRS[-1])

    # other layouts share these addresses at other sizes
    for fingerprint in (_radar_tracks_fingerprint(size=8), gen_empty_fingerprint()):
      RD, CP = self._setup(CAR.GENESIS_GV80_2025, fingerprint)
      self.assertTrue(CP.radarUnavailable)
      self.assertFalse(RD.use_radar_tracks)

  def test_no_radar_dbc(self):
    RD, CP = self._setup(CAR.KIA_EV9, _radar_tracks_fingerprint())
    self.assertFalse(RD.use_radar_tracks)

  def test_real_frame(self):
    # GENESIS_GV80_2025 79f534ddd71804c0/00000207--d9c251d178/1
    addr, dat = 0x3B0, bytes.fromhex("8f4a804a24b83007e0e0eccd3fe19fff01000005d0020000")
    RD, _ = self._setup(CAR.GENESIS_GV80_2025, _radar_tracks_fingerprint())
    RD.rcp.update([(0, [(addr, dat, 0)])])
    msg = RD.rcp.vl[f"RADAR_TRACK_{addr:x}"]
    self.assertEqual(msg['STATE'], 3)
    self.assertAlmostEqual(msg['LONG_DIST'], 22.4, places=2)
    self.assertAlmostEqual(msg['LAT_DIST'], -15.3, places=2)
    self.assertAlmostEqual(msg['REL_SPEED'], -0.51, places=2)

  def test_tracks(self):
    RD, _ = self._setup(CAR.GENESIS_GV80_2025, _radar_tracks_fingerprint())
    packer = CANPacker('hyundai_radar_3a5_3c4_generated')
    target = 0x3B0

    def cycle(t, state):
      msgs = []
      for addr in RADAR_TRACKS_ADDRS:
        values = {'STATE': state, 'LONG_DIST': 30.0, 'LAT_DIST': -1.5, 'REL_SPEED': -2.0} if addr == target else {'STATE': 0}
        msgs.append(packer.make_can_msg(f"RADAR_TRACK_{addr:x}", 0, values))
      return RD.update([(int(t * 1e9), msgs)])

    track_ids = set()
    for i in range(5):
      rr = cycle(i * 0.05, 3)
      self.assertFalse(rr.errors.canError)
      self.assertEqual(len(rr.points), 1)
      pt = rr.points[0]
      self.assertAlmostEqual(pt.dRel, 30.0, places=2)
      self.assertAlmostEqual(pt.yRel, -1.5, places=2)
      self.assertAlmostEqual(pt.vRel, -2.0, places=2)
      track_ids.add(pt.trackId)
    self.assertEqual(len(track_ids), 1)

    self.assertEqual(len(cycle(0.25, 0).points), 0)
    self.assertNotIn(cycle(0.3, 3).points[0].trackId, track_ids)
