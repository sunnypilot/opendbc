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
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD_LKA_STEER_MSG_ALT | HyundaiFlags.CANFD_LKA_STEER_MSG)
  return AdasDrvEcuInterceptor(CP, SimpleNamespace(flags=HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR | HyundaiFlagsSP.CANFD_ANGLE_STEERING))


def lkas_alt(camera, enabled=True, angle=5.0, msg_161=None, ic=None):
  ic = ic or interceptor()
  (addr, dat, bus), = ic.create_lkas_alt(CANPacker(DBC_NAME), SimpleNamespace(ACAN=0), camera, msg_161 or {}, enabled, angle)
  parser = CANParser(DBC_NAME, [("LKAS_ALT", 0)], 0)
  parser.update([(0, [(addr, dat, bus)])])
  return parser.vl["LKAS_ALT"], dat


class TestLkasAlt(unittest.TestCase):
  def test_camera_fields_pass(self):
    out, _ = lkas_alt(CAMERA)
    for k in ("FCA_ESA_CtrlSta", "LKA_StrSnd", "LKA_HndsoffSnd", "LKA_LHLnWrnSta", "ToiFltSta"):
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

  def _drive(self, ic, frames, icon=0, fresh=True):
    presses = []
    for _ in range(frames):
      ic.n = getattr(ic, "n", 0) + 1
      msg_161 = {"COUNTER": ic.n // 5, "LFA_ICON": icon} if fresh else {"COUNTER": 0, "LFA_ICON": icon}
      presses.append(int(lkas_alt(CAMERA, msg_161=msg_161, ic=ic)[0]["LFA_BUTTON"]))
    return presses

  def test_lfa_pressed_until_dcu_lfa_shows(self):
    ic = interceptor()
    presses = self._drive(ic, 200)
    self.assertEqual(sum(presses), 7, "20 ms every 0.5 s, from the first 0x161 counter change")
    starts = [i for i in range(1, 200) if presses[i] and not presses[i - 1]]
    self.assertEqual([b - a for a, b in zip(starts, starts[1:], strict=False)], [50, 50, 50])
    self.assertEqual(sum(self._drive(ic, 1, icon=1)), 0)
    for icon in (0, 2, 3):
      self.assertEqual(sum(self._drive(ic, 200, icon=icon)), 0, f"never again, icon {icon}")

  def test_camera_button_never_passes(self):
    ic = interceptor()
    self._drive(ic, 5, icon=1)
    out, _ = lkas_alt({**CAMERA, "LFA_BUTTON": 1}, msg_161={"COUNTER": 99, "LFA_ICON": 1}, ic=ic)
    self.assertEqual(out["LFA_BUTTON"], 0)

  def test_no_press_without_dcu_161(self):
    ic = interceptor()
    self.assertEqual(sum(self._drive(ic, 200, fresh=False)), 0, "stale or not diverted yet")
    self.assertEqual(sum(int(lkas_alt(CAMERA, ic=ic)[0]["LFA_BUTTON"]) for _ in range(100)), 0, "no 0x161 at all")

  def test_without_camera(self):
    out, _ = lkas_alt({})
    assert out["LFA_BUTTON"] == 0 and out["LKAS_ANGLE_ACTIVE"] == 1
