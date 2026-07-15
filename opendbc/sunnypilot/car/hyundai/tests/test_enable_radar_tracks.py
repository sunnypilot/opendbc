import unittest
from unittest.mock import patch

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.enable_radar_tracks import CONFIG_DATA_ID, DEFAULT_CONFIG, TRACKS_ENABLED_CONFIG, enable_radar_tracks
from opendbc.sunnypilot.car.interfaces import _initialize_radar_tracks
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


class TestInitializeRadarTracks(unittest.TestCase):
  @staticmethod
  def _params(flags: int = 0, radar_unavailable: bool = True):
    CP = structs.CarParams(brand="hyundai", flags=flags, radarUnavailable=radar_unavailable)
    CP_SP = structs.CarParamsSP(flags=HyundaiFlagsSP.RADAR_FULL_RADAR.value)
    return CP, CP_SP

  @patch("opendbc.sunnypilot.car.interfaces.hyundai_enable_radar_tracks")
  def test_enables_requested_classic_can_tracks_when_range_not_detected(self, enable_tracks):
    CP, CP_SP = self._params()
    enable_tracks.return_value = True

    _initialize_radar_tracks(CP, CP_SP)

    enable_tracks.assert_called_once_with(None, None, bus=0, addr=0x7d0)
    self.assertFalse(CP.radarUnavailable)

  @patch("opendbc.sunnypilot.car.interfaces.hyundai_enable_radar_tracks")
  def test_does_not_enable_when_not_requested(self, enable_tracks):
    CP, CP_SP = self._params()
    CP_SP.flags = HyundaiFlagsSP.RADAR_OFF.value

    _initialize_radar_tracks(CP, CP_SP)

    enable_tracks.assert_not_called()

  @patch("opendbc.sunnypilot.car.interfaces.hyundai_enable_radar_tracks")
  def test_does_not_enable_when_range_detected(self, enable_tracks):
    CP, CP_SP = self._params(HyundaiFlags.RADAR_TRACKS_DETECTED.value)

    _initialize_radar_tracks(CP, CP_SP)

    enable_tracks.assert_not_called()

  @patch("opendbc.sunnypilot.car.interfaces.hyundai_enable_radar_tracks")
  def test_does_not_enable_on_canfd(self, enable_tracks):
    CP, CP_SP = self._params(HyundaiFlags.CANFD.value)

    _initialize_radar_tracks(CP, CP_SP)

    enable_tracks.assert_not_called()

  @patch("opendbc.sunnypilot.car.interfaces.hyundai_enable_radar_tracks")
  def test_failed_enable_preserves_radar_unavailable(self, enable_tracks):
    CP, CP_SP = self._params()
    enable_tracks.return_value = False

    _initialize_radar_tracks(CP, CP_SP)

    self.assertTrue(CP.radarUnavailable)


class TestEnableRadarTracks(unittest.TestCase):
  @staticmethod
  def _run_with_config(config: bytes):
    with patch("opendbc.sunnypilot.car.hyundai.enable_radar_tracks.IsoTpParallelQuery") as query:
      query.return_value.get_data.side_effect = [
        {(0x7d0, None): b""},
        {(0x7d0, None): CONFIG_DATA_ID + config},
        {},
      ]
      enabled = enable_radar_tracks(None, None, retry=1)
      return enabled, query

  def test_known_default_config_is_enabled(self):
    enabled, query = self._run_with_config(DEFAULT_CONFIG)

    self.assertTrue(enabled)
    self.assertEqual(query.call_count, 3)

  def test_known_enabled_config_is_not_rewritten(self):
    enabled, query = self._run_with_config(TRACKS_ENABLED_CONFIG)

    self.assertTrue(enabled)
    self.assertEqual(query.call_count, 2)

  def test_unknown_config_is_not_written(self):
    enabled, query = self._run_with_config(b"\x01\x02\x03\x04\x05\x06")

    self.assertFalse(enabled)
    self.assertEqual(query.call_count, 2)


if __name__ == "__main__":
  unittest.main()
