import re
import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.values import CAR, FSD_14_FW, is_fsd_14_fw, match_fw_to_car_fuzzy

Ecu = CarParams.Ecu

# Fields prefixed unknown_* we observe structurally but don't know the meaning of.
# Only `platform` has evidence-backed semantic meaning (matches car_model in FW_VERSIONS).
#
# unknown_prefix is everything before the comma; we don't split it because we don't know what its
# parts mean, but observed shape is: <family>_<package>_<triplet> (<build>), e.g.
#   TeMYG4 _ Main     _ 0.0.0 (78)     or     TeM3 _ SP_XP002p2 _ 0.0.0 (23)
#   family   package    triplet build           family  package    triplet build
#
# After the comma, the version string decomposes into:
#   platform             : E/Y/X = car model (Model 3 / Y / X). The only field with known meaning.
#   variant_code         : differentiator WITHIN a platform — hardware/trim/calibration bits packed
#                          into <digit?><letters?><3-digit series>, e.g. '4HP015', '4003', 'L014',
#                          'PR003'. We don't fully know what the parts mean individually, but the
#                          whole string identifies a specific variant within the car model.
#   software_major/minor : numeric components after the first '.' — conventional release numbers.
#                          minor is optional (e.g. 'E4S014.27' has no minor).
#
# Suspected (not confirmed): for M3/MY, `TeM3_*` outer + no-leading-digit variant_code == HW3, and
# `TeMYG4_*` outer + leading-'4' variant_code == HW4 (the 'G4' in TeMYG4 likely denotes Gen 4).
#
# Example full parse of 'TeMYG4_Main_0.0.0 (78),E4HP015.05.0':
#   unknown_prefix='TeMYG4_Main_0.0.0 (78)'
#   platform=E  variant_code=4HP015  software_major=05  software_minor=0
FW_RE = re.compile(
  rb'^(?P<unknown_prefix>.+),' +
  rb'(?P<platform>[EYX])' +
  rb'(?P<variant_code>\d?[A-Z]*\d{3})' +
  rb'\.(?P<software_major>\d+)' +
  rb'(?:\.(?P<software_minor>\d+))?$'
)

PLATFORM_TO_CAR = {
  b'E': CAR.TESLA_MODEL_3,
  b'Y': CAR.TESLA_MODEL_Y,
  b'X': CAR.TESLA_MODEL_X,
}

# Hypothesized FSD 14 profile, in terms of variant_code bookends (given software_major >= 4):
#   M3: variant_code starts with '4H',  ends with '015'
#   MY: variant_code starts with '4',   ends with '003'
# Older series (M3 '014', MY '002') are never FSD 14.
FSD_14_FW_RULE = {
  CAR.TESLA_MODEL_3: (b'4H', b'015'),
  CAR.TESLA_MODEL_Y: (b'4',  b'003'),
}


class TestTeslaFingerprint(unittest.TestCase):
  def test_fw_platform_code(self):
    # Every EPS FW must parse and its platform letter must match the car it's filed under.
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)

        assert m is not None, f"Unparsable FW: {fw}"
        assert PLATFORM_TO_CAR[m['platform']] == car_model, f"Platform letter {m['platform']!r} != {car_model.value}: {fw}"

  def test_fsd_14_fw(self):
    for car_model, ecus in FW_VERSIONS.items():
      if car_model not in FSD_14_FW_RULE:
        continue

      variant_prefix, variant_suffix = FSD_14_FW_RULE[car_model]
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)
        assert m is not None, f"Unparsable FW: {fw}"

        is_fsd_14 = fw in FSD_14_FW.get(car_model, [])
        expected = (
          m['variant_code'].startswith(variant_prefix)
          and m['variant_code'].endswith(variant_suffix)
          and int(m['software_major']) >= 4
        )
        assert is_fsd_14 == expected, f"{fw}"
        assert is_fsd_14_fw(car_model, fw) == expected, f"{fw}"

    # A newer OTA with the same evidence-backed platform/variant structure
    # should not require another exact-list entry.
    assert is_fsd_14_fw(CAR.TESLA_MODEL_Y, b'TeMYG4_Main_0.0.0 (99),Y4003.99.1')

  def test_unknown_model_y_hw4_fuzzy_fw(self):
    live_fw = {(0x730, None): {b'TeMYG4_Main_0.0.0 (99),Y4003.99.1'}}
    assert match_fw_to_car_fuzzy(live_fw, "", {}) == {str(CAR.TESLA_MODEL_Y)}

  def test_tesla_fuzzy_fw_rejects_wrong_address_or_mixed_platforms(self):
    fw_y = b'TeMYG4_Main_0.0.0 (99),Y4003.99.1'
    fw_e = b'TeMYG4_Main_0.0.0 (99),E4H015.99.1'
    assert match_fw_to_car_fuzzy({(0x731, None): {fw_y}}, "", {}) == set()
    assert match_fw_to_car_fuzzy({(0x730, None): {fw_y, fw_e}}, "", {}) == set()

  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_no_radar_car(self):
    # Model X doesn't have radar DBC defined, should always be unavailable
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_X, fingerprint, [], False, False, False)
      assert CP.radarUnavailable  # Always unavailable since no radar DBC


class TestTeslaNativeSteeringArbitration(unittest.TestCase):
  def setUp(self):
    # The state helpers only depend on these arbitration fields.
    self.cs = CarState.__new__(CarState)
    self.cs.stock_lkas_passthrough = False
    self.cs.stock_lkas_prev = False

  def test_stock_lkas_before_engagement_starts_passthrough(self):
    self.cs.update_stock_lkas_state(True, cruise_was_enabled=False)
    self.assertTrue(self.cs.stock_lkas_passthrough)

  def test_stock_lkas_cannot_take_over_after_engagement(self):
    self.cs.update_stock_lkas_state(True, cruise_was_enabled=True)
    self.assertFalse(self.cs.stock_lkas_passthrough)

  def test_stock_lkas_release_ends_passthrough(self):
    self.cs.update_stock_lkas_state(True, cruise_was_enabled=False)
    self.cs.update_stock_lkas_state(False, cruise_was_enabled=True)
    self.assertFalse(self.cs.stock_lkas_passthrough)
