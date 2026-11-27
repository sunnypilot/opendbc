"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import unittest
from dataclasses import dataclass, field
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC as DBC_PARSE
from opendbc.car import structs
from opendbc.car.hyundai.hyundaican import create_acc_commands
from opendbc.car.hyundai.hyundaicanfd import create_acc_control
from opendbc.car.hyundai.values import DBC, CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.lead_data_ext import CanLeadData, CanFdLeadData
from opendbc.sunnypilot.car.hyundai.longitudinal import controller as ctl
from opendbc.sunnypilot.car.hyundai.longitudinal.config import SCC14Family
from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalController
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_longitudinal_tune
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState


@dataclass
class CP:
  carFingerprint: str = "KIA_NIRO_EV"
  flags: int = 0
  radarUnavailable: bool = False


@dataclass
class Actuators:
  accel: float = 0.0
  longControlState = LongCtrlState


@dataclass
class CC:
  actuators: Actuators = field(default_factory=lambda: Actuators())
  longActive: bool = True
  hudControl: SimpleNamespace = field(default_factory=lambda: SimpleNamespace(leadDistanceBars=2))


@dataclass
class Out:
  vEgo: float = 0.0
  aEgo: float = 0.0


@dataclass
class CS:
  out: Out = field(default_factory=lambda: Out())


class ControllerTest(unittest.TestCase):
  def setUp(self):
    self.CS = CS(out=Out(vEgo=15.0))
    self.CC = CC()
    self.drive("KIA_NIRO_EV")

  def drive(self, car, flags=HyundaiFlagsSP.LONG_TUNING, cp_flags=0):
    self.CP = CP(carFingerprint=car, flags=cp_flags)
    self.CP_SP = CP(flags=flags)
    self.controller = LongitudinalController(self.CP, self.CP_SP)

  def step(self, accel, state=LongCtrlState.pid):
    self.CC.actuators.accel, self.CC.actuators.longControlState = accel, state
    self.controller.update(self.CC, self.CS)
    return self.controller.tuning

  def hold(self, accel):
    self.controller.accel_last = self.controller.actual_accel = accel
    self.controller.accel_cmd_hist.extend([accel] * 5)


class TestLongitudinalTuning(ControllerTest):
  def test_enabled(self):
    self.assertTrue(self.controller.enabled)
    self.CP_SP.flags = 0
    self.assertFalse(self.controller.enabled)

  def test_stopping_follows_long_control(self):
    for flags in (0, HyundaiFlagsSP.LONG_TUNING):
      self.CP_SP.flags = flags
      self.assertTrue(self.step(-0.3, LongCtrlState.stopping).stopping)
      self.assertFalse(self.step(-0.3, LongCtrlState.pid).stopping)

  def test_off(self):
    self.drive("GENESIS_G70", 0)
    t = self.step(1.5)
    self.assertEqual((t.jerk_upper, t.jerk_lower, t.desired_accel, t.actual_accel, t.send_scc14), (3.0, 5.0, 1.5, 1.5, True))
    self.assertEqual(self.step(-0.5, LongCtrlState.stopping).jerk_upper, 1.0)

  def test_integrator_runs_every_scc_frame(self):
    self.CS.out.vEgo = 5.0
    self.controller.lead_one = SimpleNamespace(status=True, dRel=20.0)
    t = self.step(-2.0)
    self.assertAlmostEqual(t.jerk_lower, 2.0 / ctl.ESC_CLOSE_T_LOWER)
    self.assertAlmostEqual(t.actual_accel, -t.jerk_lower * ctl.DT)
    self.assertEqual(t.desired_accel, -2.0)

  def test_inactive(self):
    self.step(-1.0)
    self.CC.longActive = False
    t = self.step(-2.0)
    self.assertEqual((t.jerk_upper, t.jerk_lower, t.comfort_band_upper, t.comfort_band_lower, t.actual_accel), (0.0,) * 5)
    self.assertEqual(len(self.controller.accel_cmd_hist), 0)

  def test_floors(self):
    t = self.step(0.0)
    self.assertEqual((t.jerk_upper, t.jerk_lower), (self.controller.car_config.jerk_upper_floor, ctl.ESC_JERK_LOWER))
    self.controller.lead_one = SimpleNamespace(status=True, dRel=30.0)  # 2 bars at 15 m/s: within 33.2 m
    self.assertEqual(self.step(0.0).jerk_lower, ctl.ESC_JERK_LOWER_LEAD)
    self.controller.lead_one = SimpleNamespace(status=True, dRel=36.0)
    self.assertEqual(self.step(0.0).jerk_lower, ctl.ESC_JERK_LOWER)
    self.CC.hudControl.leadDistanceBars = 3
    self.assertEqual(self.step(0.0).jerk_lower, ctl.ESC_JERK_LOWER_LEAD)

  def test_rate_and_gap(self):
    self.assertEqual(self.step(-0.1).jerk_lower, ctl.ESC_JERK_LOWER)
    self.controller.accel_last = self.controller.actual_accel = -1.0
    self.controller.accel_cmd_hist.extend([-0.8, -0.84, -0.88, -0.92, -0.96])
    self.assertAlmostEqual(self.step(-1.0).jerk_lower, 2.0)
    self.hold(0.0)
    self.controller.accel_cmd_hist.extend([-1.5] * 5)
    self.assertAlmostEqual(self.step(-1.5).jerk_lower, 1.5 / ctl.ESC_CLOSE_T_LOWER)
    self.controller.accel_last = self.controller.actual_accel = 0.0
    self.assertAlmostEqual(self.step(-3.0).jerk_lower, 5.0 - 2.5 * (15.0 - 5.0) / 15.0)
    self.controller.accel_last = self.controller.actual_accel = 0.0
    self.assertEqual(self.step(2.0, LongCtrlState.starting).jerk_upper, 0.5)

  def test_bands(self):
    t = self.step(-1.2)
    self.assertEqual((t.comfort_band_upper, t.comfort_band_lower), (0.0, 0.0))
    self.controller.lead_one = SimpleNamespace(status=True, dRel=40.0)
    for a, lower, upper in ((-3.0, 0.07, 0.0), (-1.25, 0.19, 0.0), (0.0, 0.105, 0.0), (1.0, 0.0, 0.085)):
      self.hold(a)
      t = self.step(a)
      self.assertAlmostEqual(t.comfort_band_lower, lower)
      self.assertAlmostEqual(t.comfort_band_upper, upper)

  def test_stopping(self):
    self.CS.out.vEgo = 0.25
    self.controller.lead_one = SimpleNamespace(status=True, dRel=5.0)
    self.hold(-0.6)
    t = self.step(-0.7, LongCtrlState.stopping)
    floor = self.controller.car_config.jerk_upper_floor
    self.assertEqual((t.stopping, t.desired_accel, t.jerk_upper, t.comfort_band_lower), (True, 0.0, floor, 0.0))
    self.assertAlmostEqual(t.actual_accel, -0.6 + floor * ctl.DT)
    for _ in range(200):
      t = self.step(-0.7, LongCtrlState.stopping)
    self.assertEqual(t.actual_accel, 0.0)

  def test_radar_unavailable(self):
    self.CP.radarUnavailable = True
    t = self.step(-1.0)
    self.assertEqual((t.jerk_lower, t.actual_accel), (5.0, -1.0))

  def test_jerk_upper_floor_per_car(self):
    for car, floor in (("KIA_NIRO_EV", 0.3), ("HYUNDAI_SONATA", 0.5), ("HYUNDAI_PALISADE", 1.0), ("GENESIS_GV80", 0.2)):
      self.drive(car)
      self.assertEqual(self.step(0.0).jerk_upper, floor)

  def test_lower_floor_per_car(self):
    self.drive("KIA_STINGER_2022")
    self.controller.lead_one = SimpleNamespace(status=True, dRel=60.0)
    self.assertEqual(self.step(0.0).jerk_lower, 0.5)
    self.controller.lead_one = SimpleNamespace(status=True, dRel=25.0)
    self.assertEqual(self.step(0.0).jerk_lower, 2.5)

  def test_actuator_delay(self):
    for car in ("KIA_NIRO_EV", "HYUNDAI_SONATA", "HYUNDAI_PALISADE"):
      cp = SimpleNamespace(carFingerprint=car, longitudinalActuatorDelay=0.0)
      get_longitudinal_tune(cp)
      self.assertEqual(cp.longitudinalActuatorDelay, 0.5)


class TestSCC14Families(ControllerTest):
  def drive(self, car, flags=HyundaiFlagsSP.LONG_TUNING, cp_flags=0):
    super().drive(car, flags, cp_flags)
    self.controller.lead_one = SimpleNamespace(status=True, dRel=25.0, vRel=0.0)

  def test_rate(self):
    t = self.step(-1.0)
    self.assertEqual(self.controller.scc14, SCC14Family.RATE)
    self.assertEqual(t.jerk_lower, 1.0 / ctl.ESC_CLOSE_T_LOWER)
    self.assertAlmostEqual(t.actual_accel, -1.0 / ctl.ESC_CLOSE_T_LOWER * ctl.DT)
    self.assertGreater(t.comfort_band_lower, 0.0)
    self.assertTrue(t.send_scc14)

  def test_fixed(self):
    self.drive("HYUNDAI_CUSTIN_1ST_GEN")
    t = self.step(-2.0)
    self.assertEqual((t.jerk_upper, t.jerk_lower, t.comfort_band_upper, t.comfort_band_lower), (1.0, 0.5, 0.0, 0.0))
    self.assertAlmostEqual(t.actual_accel, -0.5 * ctl.DT)
    self.assertAlmostEqual(self.step(1.0).actual_accel, -0.5 * ctl.DT + 1.0 * ctl.DT)
    self.assertTrue(t.send_scc14)

  def test_zeros(self):
    for car in ("HYUNDAI_IONIQ_EV_2020", "HYUNDAI_AZERA_6TH_GEN", "GENESIS_G90", "KIA_NIRO_HEV_2021"):
      self.drive(car)
      self.CS.out.vEgo = 15.0
      t = self.step(-2.0)
      self.assertEqual((t.jerk_upper, t.jerk_lower, t.comfort_band_upper, t.comfort_band_lower), (0.0, 0.0, 0.0, 0.0))
      self.assertEqual((t.desired_accel, t.actual_accel, t.send_scc14), (-2.0, -2.0, True))
      self.CS.out.vEgo = 0.25
      t = self.step(-1.7, LongCtrlState.stopping)
      self.assertEqual((t.stopping, t.desired_accel, t.actual_accel), (True, 0.0, 0.0))

  def test_none(self):
    for car in ("GENESIS_G70", "HYUNDAI_TUCSON", "HYUNDAI_SONATA_LF", "HYUNDAI_IONIQ_EV_LTD", "KIA_OPTIMA_G4", "KIA_OPTIMA_G4_FL",
                "KIA_SORENTO", "KIA_STINGER", "GENESIS_G80", "HYUNDAI_IONIQ"):
      self.drive(car)
      t = self.step(-2.0)
      self.assertFalse(t.send_scc14)
      self.assertEqual((t.desired_accel, t.actual_accel), (-2.0, -2.0))

  def test_stepped(self):
    self.drive("HYUNDAI_TUCSON_4TH_GEN", cp_flags=HyundaiFlags.CANFD)
    self.controller.accel_last = self.controller.actual_accel = 0.5
    t = self.step(0.8)
    self.assertEqual((t.jerk_upper, t.jerk_lower), (0.6, 1.7))
    self.assertAlmostEqual(t.actual_accel, 0.5 + 0.6 * ctl.DT)
    self.controller.accel_last = self.controller.actual_accel = -0.5
    t = self.step(-1.0)
    self.assertEqual((t.jerk_upper, t.jerk_lower), (1.2, 1.1))
    self.assertAlmostEqual(t.actual_accel, -0.5 - 1.1 * ctl.DT)

  def test_packed(self):
    packer = CANPacker(DBC[CAR.KIA_NIRO_EV]['pt'])
    lead = CanLeadData(0, 0.0, 0.0, False)
    for send in (True, False):
      msgs = create_acc_commands(packer, True, -2.0, 1.0, 0, lead, SimpleNamespace(leadDistanceBars=2), 20, False, False, True,
                                 SimpleNamespace(flags=HyundaiFlags.USE_FCA), True, ctl.LongitudinalState(send_scc14=send))
      self.assertEqual(0x389 in [addr for addr, _, _ in msgs], send)


class TestCanFdBands(ControllerTest):
  def test_level_per_car(self):
    for car, bands in (("HYUNDAI_TUCSON_4TH_GEN", (0.1, 0.1)), ("GENESIS_G80_2ND_GEN_FL", (0.2, 0.2)), ("KIA_EV6", (0.0, 0.0))):
      self.drive(car, cp_flags=HyundaiFlags.CANFD)
      self.controller.lead_one = SimpleNamespace(status=True, dRel=30.0, vRel=-1.0)
      for accel in (-1.5, 0.0, 1.0):
        t = self.step(accel)
        self.assertEqual((t.comfort_band_upper, t.comfort_band_lower), bands)

  def test_no_lead_or_pulling_away(self):
    self.drive("HYUNDAI_TUCSON_4TH_GEN", cp_flags=HyundaiFlags.CANFD)
    for lead in (None, SimpleNamespace(status=False, dRel=30.0, vRel=0.0), SimpleNamespace(status=True, dRel=30.0, vRel=1.5)):
      self.controller.lead_one = lead
      t = self.step(-1.0)
      self.assertEqual((t.comfort_band_upper, t.comfort_band_lower), (0.0, 0.0))

  def test_standstill(self):
    self.drive("HYUNDAI_TUCSON_4TH_GEN", cp_flags=HyundaiFlags.CANFD)
    self.CS.out.vEgo = 0.0
    self.controller.lead_one = SimpleNamespace(status=True, dRel=4.0, vRel=0.0)
    t = self.step(-0.5, LongCtrlState.stopping)
    self.assertEqual((t.stopping, t.comfort_band_lower), (True, 0.1))

  def test_packed(self):
    packer = CANPacker(DBC[CAR.HYUNDAI_IONIQ_6]['pt'])
    sigs = DBC_PARSE('hyundai_canfd_generated').addr_to_msg[0x1A0].sigs
    tuning = ctl.LongitudinalState(comfort_band_upper=0.04, comfort_band_lower=0.16)
    _, dat, _ = create_acc_control(packer, SimpleNamespace(ECAN=0), True, 0.0, -1.0, False, False, 20, SimpleNamespace(leadDistanceBars=2),
                                   CanFdLeadData(2, 30.0, -1.0, True), True, tuning)
    self.assertAlmostEqual(sigs['ComfortBandUpper'].get_raw_value(dat) * 0.02, 0.04)
    self.assertAlmostEqual(sigs['ComfortBandLower'].get_raw_value(dat) * 0.02, 0.16)


if __name__ == "__main__":
  unittest.main()
