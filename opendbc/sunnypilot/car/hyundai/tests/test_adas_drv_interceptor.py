"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.hyundai.interceptors.adas_drv_interceptor import AdasDrvEcuInterceptor
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

DBC_NAME = "hyundai_canfd_generated"
CAMERA = {"LFA_BUTTON": 1, "FCA_ESA_CtrlSta": 1, "LKA_StrSnd": 2, "LKA_HndsoffSnd": 1, "LKA_LHLnWrnSta": 1,
          "ToiFltSta": 1, "LKA_SysIndReq": 2, "StrTqReqVal": 100, "ActToiSta": 1, "LKAS_ANGLE_ACTIVE": 2,
          "ADAS_StrAnglReqVal": 30.0, "ADAS_ACIAnglTqRedcGainVal": 0.5, "CHECKSUM": 1234, "COUNTER": 77}


def interceptor(long=True):
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD_LKA_STEER_MSG_ALT | HyundaiFlags.CANFD_LKA_STEER_MSG, openpilotLongitudinalControl=long)
  return AdasDrvEcuInterceptor(CP, SimpleNamespace(flags=HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR | HyundaiFlagsSP.CANFD_ANGLE_STEERING))


class TestInterceptOpt(unittest.TestCase):
  def test_nav_only_with_long(self):
    for long, features in ((True, 0x4F), (False, 0x03)):
      (_, dat, _), = interceptor(long).create_adas_drv_intercept_msg(CANPacker(DBC_NAME), SimpleNamespace(ACAN=0))
      self.assertEqual(dat[7], features, f"long {long}")


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


def dcu_emergency(angle, long, ecan=(), acan=(), msg_161=None):
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD | HyundaiFlags.CANFD_LKA_STEER_MSG | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT,
                       openpilotLongitudinalControl=long, carFingerprint=CAR.KIA_CARNIVAL_4TH_GEN, safetyConfigs=[None])
  CP_SP = SimpleNamespace(flags=HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR | (HyundaiFlagsSP.CANFD_ANGLE_STEERING if angle else 0))
  CS = CarStateExt(CP, CP_SP)
  CS.msg_161 = msg_161 or {}
  parsers = CarStateExt.get_parser(CP, CP_SP)
  packer, CAN = CANPacker(DBC_NAME), CanBus(CP)
  frames = [packer.make_can_msg(n, CAN.ECAN, v) for n, v in ecan] + [packer.make_can_msg(n, CAN.ACAN, v) for n, v in acan]
  for cp in parsers.values():
    cp.update([(0, frames)])
  ret_sp = structs.CarStateSP()
  CS.update_dcu_emergency(ret_sp, parsers)
  return ret_sp, parsers


class TestDcuEmergency(unittest.TestCase):
  def test_lfa_copy_bus(self):
    for angle, long, on_ecan in ((True, True, True), (True, False, True), (False, False, True), (False, True, False)):
      frames = {"ecan" if on_ecan else "acan": [("LFA", {"FCA_LS_WrnSta": 1})]}
      ret_sp, parsers = dcu_emergency(angle, long, **frames)
      assert "LFA" in parsers[Bus.alt if on_ecan else Bus.adas].vl, (angle, long)
      assert ret_sp.stockSteerIntervention, (angle, long)

  def test_triggers(self):
    for values in ({"FCA_LO_WrnSta": 1}, {"FCA_ESA_WrnSta": 1}, {"FCA_ESA_CtrlSta": 1}, {"BCA_Rear_WrnSta": 4}, {"ELK_SymbDisp": 3}):
      assert dcu_emergency(True, True, ecan=[("LFA", values)])[0].stockSteerIntervention, values
    assert not dcu_emergency(True, True, ecan=[("LFA", {"ELK_SymbDisp": 1})])[0].stockSteerIntervention
    for values in ({"FCA_ESA_ActvSta": 1}, {"FCA_ESA_TqBstGainVal": 0.1}):
      assert dcu_emergency(True, True, acan=[("LFA_ALT", values)])[0].stockSteerIntervention, values
    assert not dcu_emergency(True, True, msg_161={"ALERTS_1": 21})[0].stockSteerIntervention
