"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest
from unittest.mock import MagicMock

from opendbc.car import structs
from opendbc.car.subaru.values import SubaruFlags
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.subaru.values_ext import SubaruSafetyFlagsSP


def _run_setup(params: dict, brand: str = 'subaru', alpha_long_available: bool = True):
  CP = structs.CarParams()
  CP.brand = brand
  CP.alphaLongitudinalAvailable = alpha_long_available
  CP_SP = structs.CarParamsSP()
  setup_interfaces(MagicMock(), CP, CP_SP, params_list=[params])
  return CP, CP_SP


class TestBrakeInterceptInit(unittest.TestCase):
  def test_flag_not_set_when_param_off(self):
    CP, CP_SP = _run_setup({"SubaruAutoVehicleHold": "0"})
    self.assertFalse(CP.flags & SubaruFlags.BRAKE_HOLD)
    self.assertFalse(CP_SP.safetyParam & SubaruSafetyFlagsSP.BRAKE_INTERCEPT)

  def test_flag_set_when_param_on(self):
    CP, CP_SP = _run_setup({"SubaruAutoVehicleHold": "1"})
    self.assertTrue(CP.flags & SubaruFlags.BRAKE_HOLD)
    self.assertTrue(CP_SP.safetyParam & SubaruSafetyFlagsSP.BRAKE_INTERCEPT)

  def test_flag_not_set_when_ineligible_even_if_param_on(self):
    CP, CP_SP = _run_setup({"SubaruAutoVehicleHold": "1"}, alpha_long_available=False)
    self.assertFalse(CP.flags & SubaruFlags.BRAKE_HOLD)
    self.assertFalse(CP_SP.safetyParam & SubaruSafetyFlagsSP.BRAKE_INTERCEPT)

  def test_brake_intercept_preserves_sng_bit(self):
    # BRAKE_INTERCEPT is OR-added, must not clobber the SnG safety bit
    _, CP_SP = _run_setup({"SubaruStopAndGo": "1", "SubaruAutoVehicleHold": "1"})
    self.assertTrue(CP_SP.safetyParam & SubaruSafetyFlagsSP.STOP_AND_GO)
    self.assertTrue(CP_SP.safetyParam & SubaruSafetyFlagsSP.BRAKE_INTERCEPT)

  def test_non_subaru_brand_untouched(self):
    CP, CP_SP = _run_setup({"SubaruAutoVehicleHold": "1"}, brand='toyota')
    self.assertFalse(CP.flags & SubaruFlags.BRAKE_HOLD)
    self.assertFalse(CP_SP.safetyParam & SubaruSafetyFlagsSP.BRAKE_INTERCEPT)


if __name__ == "__main__":
  unittest.main()
