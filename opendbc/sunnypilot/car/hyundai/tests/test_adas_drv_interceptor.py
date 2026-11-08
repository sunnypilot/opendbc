"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.interceptors.adas_drv_interceptor import AdasDrvEcuInterceptor
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

DBC_NAME = "hyundai_canfd_generated"
CAMERA = {"LFA_BUTTON": 1, "FCA_ESA_CtrlSta": 1, "LKA_StrSnd": 2, "LKA_HndsoffSnd": 1, "LKA_LHLnWrnSta": 1,
          "ToiFltSta": 1, "LKA_SysIndReq": 2, "StrTqReqVal": 100, "ActToiSta": 1, "LKAS_ANGLE_ACTIVE": 2,
          "ADAS_StrAnglReqVal": 30.0, "ADAS_ACIAnglTqRedcGainVal": 0.5, "CHECKSUM": 1234, "COUNTER": 77}


def interceptor():
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD_LKA_STEER_MSG_ALT | HyundaiFlags.CANFD_ANGLE_STEERING | HyundaiFlags.CANFD_LKA_STEER_MSG)
  return AdasDrvEcuInterceptor(CP, SimpleNamespace(flags=HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR))


def lkas_alt(camera, enabled=True, angle=5.0, icon=0, ic=None):
  ic = ic or interceptor()
  (addr, dat, bus), = ic.create_lkas_alt(CANPacker(DBC_NAME), SimpleNamespace(ACAN=0), camera, icon, enabled, angle)
  parser = CANParser(DBC_NAME, [("LKAS_ALT", 0)], 0)
  parser.update([(0, [(addr, dat, bus)])])
  return parser.vl["LKAS_ALT"], dat


class TestLkasAlt(unittest.TestCase):
  def test_camera_fields_pass(self):
    out, _ = lkas_alt(CAMERA)
    for k in ("LFA_BUTTON", "FCA_ESA_CtrlSta", "LKA_StrSnd", "LKA_HndsoffSnd", "LKA_LHLnWrnSta", "ToiFltSta"):
      assert out[k] == CAMERA[k], k

  def test_steering_never_engaged(self):
    out, dat = lkas_alt(CAMERA, enabled=False)
    assert out["LKAS_ANGLE_ACTIVE"] == 1
    assert out["StrTqReqVal"] == 0 and out["ActToiSta"] == 0 and out["ADAS_ACIAnglTqRedcGainVal"] == 0
    assert out["ADAS_StrAnglReqVal"] == 5.0
    assert out["LKA_SysIndReq"] == 1
    assert (dat[9] >> 4) & 0x3 == 1 and dat[12] == 0, "panda safety accepts it"

  def test_counter_not_copied(self):
    _, dat = lkas_alt(CAMERA)
    assert dat[2] != CAMERA["COUNTER"]

  def test_lfa_button_dropped_once_dcu_lfa_shows(self):
    ic = interceptor()
    press = {**CAMERA, "LFA_BUTTON": 1}
    self.assertEqual(lkas_alt(press, icon=0, ic=ic)[0]["LFA_BUTTON"], 1, "passes while hidden")
    self.assertEqual(lkas_alt(press, icon=1, ic=ic)[0]["LFA_BUTTON"], 0, "dropped once shown")
    for icon in (0, 2, 3):
      out, _ = lkas_alt(press, icon=icon, ic=ic)
      self.assertEqual(out["LFA_BUTTON"], 0, f"still dropped, icon {icon}")
      self.assertEqual(out["FCA_ESA_CtrlSta"], 1, "other camera fields still pass")

  def test_without_camera(self):
    out, _ = lkas_alt({})
    assert out["LFA_BUTTON"] == 0 and out["LKAS_ANGLE_ACTIVE"] == 1
