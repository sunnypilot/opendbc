"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.can.dbc import DBC
from opendbc.car import structs
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.hyundai.interceptors.adas_drv_interceptor import AdasDrvEcuInterceptor, AdasDrvEcuInterceptorCarController
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

DBC_NAME = "hyundai_canfd_generated"
CAMERA = {"LFA_BUTTON": 1, "FCA_ESA_CtrlSta": 1, "LKA_StrSnd": 2, "LKA_HndsoffSnd": 1, "LKA_LHLnWrnSta": 1,
          "ToiFltSta": 1, "LKA_SysIndReq": 2, "StrTqReqVal": 100, "ActToiSta": 1, "LKAS_ANGLE_ACTIVE": 2,
          "ADAS_StrAnglReqVal": 30.0, "ADAS_ACIAnglTqRedcGainVal": 0.5, "CHECKSUM": 1234, "COUNTER": 77}


def interceptor(long=True, alt=True):
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD_LKA_STEER_MSG | (HyundaiFlags.CANFD_LKA_STEER_MSG_ALT if alt else 0), openpilotLongitudinalControl=long)
  return AdasDrvEcuInterceptor(CP, SimpleNamespace(flags=HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR | HyundaiFlagsSP.CANFD_ANGLE_STEERING))


class TestInterceptOpt(unittest.TestCase):
  def test_nav_only_with_long(self):
    for long, features in ((True, 0x4F), (False, 0x03)):
      (_, dat, _), = interceptor(long).create_adas_drv_intercept_msg(CANPacker(DBC_NAME), SimpleNamespace(ACAN=0))
      self.assertEqual(dat[7], features, f"long {long}")

  def test_copied_frames_fully_defined(self):
    # the camera's 0x50 is rebuilt from its signals: a bit without one comes out 0
    msg = DBC(DBC_NAME).addr_to_msg[0x50]
    for bit in range(msg.size * 8):
      dat = bytearray(msg.size)
      dat[bit // 8] = 1 << (bit % 8)
      self.assertTrue(any(s.get_raw_value(dat) for s in msg.sigs.values()), f"{msg.name} bit {bit}")

def decode(addr, dat):
  values = {}
  for sig in DBC(DBC_NAME).addr_to_msg[addr].sigs.values():
    raw = sig.get_raw_value(dat)
    if sig.is_signed and raw >> (sig.size - 1):
      raw -= 1 << sig.size
    values[sig.name] = raw * sig.factor + sig.offset
  return values


class TestLkasTorque(unittest.TestCase):
  def test_camera_frame_with_our_request(self):
    for alt, addr, size in ((True, 0x110, 32), (False, 0x50, 16)):
      for seed in range(20):
        camera = decode(addr, random.Random(seed).randbytes(size))
        for lfa_button, lat_active in ((True, True), (False, False)):
          out_addr, dat, _ = interceptor(alt=alt).create_lkas(CANPacker(DBC_NAME), SimpleNamespace(ACAN=0), camera, lfa_button, lat_active, 150)
          out = decode(out_addr, dat)
          self.assertEqual(out_addr, addr)
          self.assertEqual((out["StrTqReqVal"], out["ActToiSta"], out["LFA_BUTTON"]), (150, int(lat_active), int(lfa_button)))
          for k, v in camera.items():
            if k not in ("CHECKSUM", "COUNTER", "StrTqReqVal", "ActToiSta", "LFA_BUTTON"):
              self.assertEqual(out[k], v, (alt, seed, k))


def lkas_alt(camera, enabled=True, angle=5.0, msg_161=None, ic=None):
  ic = ic or interceptor()
  ic.update_car_state(None)
  (addr, dat, bus), = ic.create_lkas_alt(CANPacker(DBC_NAME), SimpleNamespace(ACAN=0), camera, ic.lfa_press(msg_161 or {}), enabled, angle)
  parser = CANParser(DBC_NAME, [("LKAS_ALT", 0)], 0)
  parser.update([(0, [(addr, dat, bus)])])
  return parser.vl["LKAS_ALT"], dat


class TestLkasAlt(unittest.TestCase):
  def test_camera_fields_pass(self):
    out, _ = lkas_alt(CAMERA)
    for k in ("FCA_ESA_CtrlSta", "LKA_StrSnd", "LKA_HndsoffSnd", "LKA_LHLnWrnSta", "ToiFltSta"):
      assert out[k] == CAMERA[k], k

  def test_camera_frame_passes(self):
    # GV80, Carnival
    msg = DBC(DBC_NAME).addr_to_msg[0x110]
    ours = {"CHECKSUM", "COUNTER", "LFA_BUTTON", "LKA_OptUsmSta", "LKA_SysIndReq", "StrTqReqVal", "LKA_SysWrn", "ActToiSta", "LKA_UsmMod",
            "LKA_RcgSta", "Damping_Gain", "ADAS_StrAnglReqVal", "LKAS_ANGLE_ACTIVE", "ADAS_ACIAnglTqRedcGainVal"}
    mask = bytearray(msg.size)
    for sig in msg.sigs.values():
      if sig.name in ours:
        for bit in range(msg.size * 8):
          probe = bytearray(msg.size)
          probe[bit // 8] = 1 << (bit % 8)
          if sig.get_raw_value(probe):
            mask[bit // 8] |= 1 << (bit % 8)
    for frame in ("f771ae18a000080000120c00000900000000000000000000000000009201ffff",
                  "0b97c5006000080064020000000000000000000000000000000000009201ffff"):
      camera = bytes.fromhex(frame)
      _, dat = lkas_alt(decode(0x110, camera))
      for i, m in enumerate(mask):
        self.assertEqual(dat[i] & ~m & 0xFF, camera[i] & ~m & 0xFF, (frame, i))

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

  def test_lfa_pressed_until_adas_drv_lfa_shows(self):
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

  def test_no_press_without_adas_drv_161(self):
    ic = interceptor()
    self.assertEqual(sum(self._drive(ic, 200, fresh=False)), 0, "stale or not diverted yet")
    self.assertEqual(sum(int(lkas_alt(CAMERA, ic=ic)[0]["LFA_BUTTON"]) for _ in range(100)), 0, "no 0x161 at all")

  def test_without_camera(self):
    out, _ = lkas_alt({})
    assert out["LFA_BUTTON"] == 0 and out["LKAS_ANGLE_ACTIVE"] == 1


def adas_drv_emergency(angle, long, ecan=(), acan=(), msg_161=None, CS=None, t=1):
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD | HyundaiFlags.CANFD_LKA_STEER_MSG | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT,
                       openpilotLongitudinalControl=long, carFingerprint=CAR.KIA_CARNIVAL_4TH_GEN, safetyConfigs=[None])
  CP_SP = SimpleNamespace(flags=HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR | (HyundaiFlagsSP.CANFD_ANGLE_STEERING if angle else 0))
  if CS is None:
    CS = CarStateExt(CP, CP_SP)
    CS.parsers = CarStateExt.get_parser(CP, CP_SP)
    CS.packer = CANPacker(DBC_NAME)
  CS.msg_161 = msg_161 or {}
  CAN = CanBus(CP)
  frames = [CS.packer.make_can_msg(n, CAN.ECAN, v) for n, v in ecan] + [CS.packer.make_can_msg(n, CAN.ACAN, v) for n, v in acan]
  for cp in CS.parsers.values():
    cp.update([(t, frames)])
  ret, ret_sp = structs.CarState(), structs.CarStateSP()
  CS.update_adas_drv_emergency(ret, ret_sp, CS.parsers)
  return ret_sp, CS, ret


class TestAdasDrvEmergency(unittest.TestCase):
  def test_steering_frame_from_either_bus(self):
    for bus in ("acan", "ecan"):
      ret_sp, CS, _ = adas_drv_emergency(False, True, **{bus: [("LFA", {"FCA_LS_WrnSta": 1, "ActToiSta": 1})]})
      assert ret_sp.stockSteerIntervention and CS.adas_drv_passthrough, bus
      frames = {"ecan": [("LFA", {"FCA_LS_WrnSta": 1})], "acan": []}
      frames[bus].append(("LFA_ALT", {"ADAS_ActvACILvl2Sta": 2}))
      ret_sp, CS, _ = adas_drv_emergency(True, True, **frames)
      assert ret_sp.stockSteerIntervention and CS.adas_drv_passthrough, bus

  def test_newest_steering_frame_wins(self):
    _, CS, _ = adas_drv_emergency(False, True, acan=[("LFA", {"FCA_LS_WrnSta": 1, "ActToiSta": 1})])
    assert CS.adas_drv_passthrough
    _, CS, _ = adas_drv_emergency(False, True, ecan=[("LFA", {"ActToiSta": 1})], CS=CS, t=2)
    assert not CS.adas_drv_passthrough

  def test_passthrough_needs_emergency_and_its_steering_request(self):
    for angle, lfa, lfa_alt, passthrough in (
      (False, {"FCA_LS_WrnSta": 1, "ActToiSta": 1}, None, True), (False, {"FCA_LS_WrnSta": 1}, None, False), (False, {"ActToiSta": 1}, None, False),
      (True, {"FCA_LS_WrnSta": 1}, {"ADAS_ActvACILvl2Sta": 2}, True), (True, {"FCA_LS_WrnSta": 1}, {"ADAS_ActvACILvl2Sta": 1}, False),
      (True, {}, {"ADAS_ActvACILvl2Sta": 2}, False), (True, {}, {"ADAS_ActvACILvl2Sta": 1, "FCA_ESA_ActvSta": 1}, True),
    ):
      acan = [("LFA_ALT", lfa_alt)] if angle else [("LFA", lfa)]
      _, CS, _ = adas_drv_emergency(angle, True, ecan=[("LFA", lfa)] if angle else [], acan=acan)
      assert CS.adas_drv_passthrough == passthrough, (angle, lfa, lfa_alt)

  def test_passthrough_ends_when_steering_frame_stops(self):
    _, CS, _ = adas_drv_emergency(False, True, acan=[("LFA", {"FCA_LS_WrnSta": 1, "ActToiSta": 1})])
    seen = []
    for _ in range(4):
      seen.append(adas_drv_emergency(False, True, CS=CS)[1].adas_drv_passthrough)
    self.assertEqual(seen, [True, True, False, False])

  def test_command_restarts_after_passthrough(self):
    for angle in (True, False):
      CC = AdasDrvEcuInterceptorCarController(interceptor().CP, SimpleNamespace(flags=HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR |
                                                                              (HyundaiFlagsSP.CANFD_ANGLE_STEERING if angle else 0)))
      CC.apply_angle_last, CC.apply_torque_last, CC.angle_filter = 10.0, 200, SimpleNamespace(x=10.0)
      restarted = []
      for passthrough in (False, True, True, False, False, False):
        CC.apply_angle_last, CC.apply_torque_last = 10.0, 200
        CC.update(SimpleNamespace(adas_drv_passthrough=passthrough, out=SimpleNamespace(steeringAngleDeg=-25.0)))
        restarted.append((CC.apply_angle_last, CC.angle_filter.x) == (-25.0, -25.0) if angle else CC.apply_torque_last == 0)
      self.assertEqual(restarted, [False, False, False, True, True, False], angle)

  def test_aeb_from_brake_request(self):
    for values, aeb in (({"SET_ME_2": 32}, False), ({"SET_ME_2": 64}, False), ({"SET_ME_2": 64, "AEB_BRAKE_REQ": 60}, True)):
      assert adas_drv_emergency(True, True, ecan=[("ADRV_0x160", values)])[2].stockAeb == aeb, values

  def test_triggers(self):
    for values in ({"FCA_LO_WrnSta": 1}, {"FCA_ESA_WrnSta": 1}, {"FCA_ESA_CtrlSta": 1}, {"BCA_Rear_WrnSta": 4}, {"ELK_SymbDisp": 3}):
      assert adas_drv_emergency(True, True, ecan=[("LFA", values)])[0].stockSteerIntervention, values
    assert not adas_drv_emergency(True, True, ecan=[("LFA", {"ELK_SymbDisp": 1})])[0].stockSteerIntervention
    for values in ({"FCA_ESA_ActvSta": 1}, {"FCA_ESA_TqBstGainVal": 0.1}):
      assert adas_drv_emergency(True, True, acan=[("LFA_ALT", values)])[0].stockSteerIntervention, values
    assert not adas_drv_emergency(True, True, msg_161={"ALERTS_1": 21})[0].stockSteerIntervention
