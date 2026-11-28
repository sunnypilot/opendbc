"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import unittest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.hyundai.lfa_alt import LFA_ALT_MSG, create_lfa_alt, mdps_aci_lv2_for_camera
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP

CCNC = (0x161, 0x162)


def get_params(cam_msgs, alpha_long=False):
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x1cf] = 8
  fingerprint[2].update(dict.fromkeys(cam_msgs, 8))
  CP = CarInterface.get_params(CAR.KIA_SPORTAGE_2026, fingerprint, [], alpha_long, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.KIA_SPORTAGE_2026, fingerprint, [], alpha_long, False, False)
  return CP, CP_SP


class TestLfaAlt(unittest.TestCase):
  def tearDown(self):
    libsafety_py.libsafety.set_current_safety_param_sp(0)

  def test_detection(self):
    for cam_msgs, expected in (((LFA_ALT_MSG, *CCNC), True), (CCNC, False), ((LFA_ALT_MSG,), False),
                               ((LFA_ALT_MSG, 0x110, *CCNC), False)):
      _, CP_SP = get_params(cam_msgs)
      assert bool(CP_SP.flags & HyundaiFlagsSP.CANFD_LFA_ALT_STEER) == expected, cam_msgs
      assert bool(CP_SP.safetyParam & HyundaiSafetyFlagsSP.CANFD_LFA_ALT_STEER) == expected, cam_msgs

  def test_mdps_for_camera(self):
    idle = {"MDPS_ADASAciActvSta_Lv2": 2, "MDPS_ADAS_AciFltSig_Lv2": 0}
    assert mdps_aci_lv2_for_camera(idle, {"ADAS_ActvACILvl2Sta": 1}) == {"MDPS_ADASAciActvSta_Lv2": 1, "MDPS_ADAS_AciFltSig_Lv2": 4}
    assert mdps_aci_lv2_for_camera(idle, {"ADAS_ActvACILvl2Sta": 2})["MDPS_ADASAciActvSta_Lv2"] == 2
    assert mdps_aci_lv2_for_camera(idle, {"ADAS_ActvACILvl2Sta": 0}) == {"MDPS_ADAS_AciFltSig_Lv2": 4}
    assert mdps_aci_lv2_for_camera(idle, {}) == {"MDPS_ADAS_AciFltSig_Lv2": 4}
    assert mdps_aci_lv2_for_camera({"MDPS_ADASAciActvSta_Lv2": 0, "MDPS_ADAS_AciFltSig_Lv2": 1}, {"ADAS_ActvACILvl2Sta": 1}) == {}

  def test_sender_passes_safety(self):
    for alpha_long in (False, True):
      CP, CP_SP = get_params((LFA_ALT_MSG, *CCNC), alpha_long)
      safety = libsafety_py.libsafety
      safety.set_current_safety_param_sp(CP_SP.safetyParam)
      safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, CP.safetyConfigs[-1].safetyParam)
      safety.init_tests()

      packer = CANPacker("hyundai_canfd_generated")
      for lat_active, gain in ((False, 0.5), (True, 0.5)):
        safety.set_controls_allowed(lat_active)
        addr, dat, bus = create_lfa_alt(packer, CanBus(CP), lat_active, 0., gain)
        assert (addr, bus) == (LFA_ALT_MSG, 0)
        assert safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)), (alpha_long, lat_active)


if __name__ == "__main__":
  unittest.main()
