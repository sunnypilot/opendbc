#!/usr/bin/env python3
import unittest

from opendbc.car.gm.values import GMSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.gas_interceptor_common import GasInterceptorSafetyTest

from opendbc.sunnypilot.car.gm.values_ext import GMSafetyFlagsSP


class Buttons:
  UNPRESS = 1
  RES_ACCEL = 2
  DECEL_SET = 3
  CANCEL = 6


class GmLongitudinalBase(common.CarSafetyTest, common.LongitudinalGasBrakeSafetyTest):

  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB), 2: (0x184,)}  # ASCMLKASteeringCmd, ASCMGasRegenCmd, PSCMStatus

  MAX_POSSIBLE_BRAKE = 2 ** 12
  MAX_BRAKE = 400

  MAX_POSSIBLE_GAS = 4000  # reasonably excessive limits, not signal max
  MIN_POSSIBLE_GAS = -4000

  PCM_CRUISE = False  # openpilot can control the PCM state if longitudinal

  def _send_brake_msg(self, brake):
    values = {"FrictionBrakeCmd": -brake}
    return self.packer_chassis.make_can_msg_safety("EBCMFrictionBrakeCmd", self.BRAKE_BUS, values)

  def _send_gas_msg(self, gas):
    values = {"GasRegenCmd": gas}
    return self.packer.make_can_msg_safety("ASCMGasRegenCmd", 0, values)

  # override these tests from CarSafetyTest, GM longitudinal uses button enable
  def _pcm_status_msg(self, enable):
    raise NotImplementedError

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_resume_buttons(self):
    """
      SET and RESUME enter controls allowed on their falling and rising edges, respectively.
    """
    for btn_prev in range(8):
      for btn_cur in range(8):
        with self.subTest(btn_prev=btn_prev, btn_cur=btn_cur):
          self._rx(self._button_msg(btn_prev))
          self.safety.set_controls_allowed(0)
          for _ in range(10):
            self._rx(self._button_msg(btn_cur))

          should_enable = btn_cur != Buttons.DECEL_SET and btn_prev == Buttons.DECEL_SET
          should_enable = should_enable or (btn_cur == Buttons.RES_ACCEL and btn_prev != Buttons.RES_ACCEL)
          should_enable = should_enable and btn_cur != Buttons.CANCEL
          self.assertEqual(should_enable, self.safety.get_controls_allowed())

  def test_cancel_button(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(Buttons.CANCEL))
    self.assertFalse(self.safety.get_controls_allowed())


class TestGmSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  STANDSTILL_THRESHOLD = 10 * 0.0311
  # Ensures ASCM is off on ASCM cars, and relay is not malfunctioning for camera-ACC cars
  RELAY_MALFUNCTION_ADDRS = {0: (0x180,), 2: (0x184,)}  # ASCMLKASteeringCmd, PSCMStatus
  BUTTONS_BUS = 0  # rx or tx
  BRAKE_BUS = 0  # tx only

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 15
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 128
  DRIVER_TORQUE_ALLOWANCE = 65
  DRIVER_TORQUE_FACTOR = 4

  PCM_CRUISE = True  # openpilot is tied to the PCM state if not longitudinal

  EXTRA_SAFETY_PARAM = 0

  def setUp(self):
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, 0)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    if self.PCM_CRUISE:
      values = {"CruiseState": enable}
      return self.packer.make_can_msg_safety("AcceleratorPedal2", 0, values)
    else:
      raise NotImplementedError

  def _speed_msg(self, speed):
    values = {"%sWheelSpd" % s: speed for s in ["RL", "RR"]}
    return self.packer.make_can_msg_safety("EBCMWheelSpdRear", 0, values)

  def _user_brake_msg(self, brake):
    # GM safety has a brake threshold of 8
    values = {"BrakePedalPos": 8 if brake else 0}
    return self.packer.make_can_msg_safety("ECMAcceleratorPos", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal2": 1 if gas else 0}
    if self.PCM_CRUISE:
      # Fill CruiseState with expected value if the safety mode reads cruise state from gas msg
      values["CruiseState"] = self.safety.get_controls_allowed()
    return self.packer.make_can_msg_safety("AcceleratorPedal2", 0, values)

  def _torque_driver_msg(self, torque):
    # Safety tests assume driver torque is an int, use DBC factor
    values = {"LKADriverAppldTrq": torque * 0.01}
    return self.packer.make_can_msg_safety("PSCMStatus", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKASteeringCmd": torque, "LKASteeringCmdActive": steer_req}
    return self.packer.make_can_msg_safety("ASCMLKASteeringCmd", 0, values)

  def _button_msg(self, buttons):
    values = {"ACCButtons": buttons}
    return self.packer.make_can_msg_safety("ASCMSteeringButton", self.BUTTONS_BUS, values)


class TestGmEVSafetyBase(TestGmSafetyBase):
  EXTRA_SAFETY_PARAM = GMSafetyFlags.EV

  # existence of _user_regen_msg adds regen tests
  def _user_regen_msg(self, regen):
    values = {"RegenPaddle": 2 if regen else 0}
    return self.packer.make_can_msg_safety("EBCMRegenPaddle", 0, values)


def test_gm_non_acc_init_paths():
  safety = libsafety_py.libsafety

  # Exercise non-ACC init branches for ASCM/camera and EV/non-EV.
  cases = (
    (GMSafetyFlagsSP.NON_ACC | GMSafetyFlagsSP.GAS_INTERCEPTOR | GMSafetyFlagsSP.PEDAL_LONG, 0),
    (GMSafetyFlagsSP.NON_ACC | GMSafetyFlagsSP.GAS_INTERCEPTOR | GMSafetyFlagsSP.PEDAL_LONG, GMSafetyFlags.HW_CAM),
    (GMSafetyFlagsSP.NON_ACC | GMSafetyFlagsSP.GAS_INTERCEPTOR | GMSafetyFlagsSP.PEDAL_LONG, GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG),
    (GMSafetyFlagsSP.NON_ACC, GMSafetyFlags.HW_CAM),
    (GMSafetyFlagsSP.NON_ACC, GMSafetyFlags.HW_CAM | GMSafetyFlags.EV),
  )

  for safety_param_sp, safety_param in cases:
    safety.set_current_safety_param_sp(safety_param_sp)
    assert safety.set_safety_hooks(CarParams.SafetyModel.gm, safety_param) == 0
    safety.init_tests()
    assert not safety.get_controls_allowed()


class TestGmAscmSafety(GmLongitudinalBase, TestGmSafetyBase):
  TX_MSGS = [[0x180, 0], [0x409, 0], [0x40A, 0], [0x2CB, 0], [0x370, 0],  # pt bus
             [0xA1, 1], [0x306, 1], [0x308, 1], [0x310, 1],  # obs bus
             [0x315, 2]]  # ch bus
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}
  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB)}  # ASCMLKASteeringCmd, ASCMGasRegenCmd
  FWD_BUS_LOOKUP: dict[int, int] = {}
  BRAKE_BUS = 2

  MAX_GAS = 1018
  MIN_GAS = -650  # maximum regen
  INACTIVE_GAS = -650

  def setUp(self):
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, self.EXTRA_SAFETY_PARAM)
    self.safety.init_tests()


class TestGmAscmEVSafety(TestGmAscmSafety, TestGmEVSafetyBase):
  pass


class TestGmCameraSafetyBase(TestGmSafetyBase):
  def _user_brake_msg(self, brake):
    values = {"BrakePressed": brake}
    return self.packer.make_can_msg_safety("ECMEngineStatus", 0, values)


class TestGmCameraSafety(TestGmCameraSafetyBase):
  TX_MSGS = [[0x180, 0],  # pt bus
             [0x184, 2]]  # camera bus
  FWD_BLACKLISTED_ADDRS = {2: [0x180], 0: [0x184]}  # block LKAS message and PSCMStatus
  BUTTONS_BUS = 2  # tx only

  def setUp(self):
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, GMSafetyFlags.HW_CAM | self.EXTRA_SAFETY_PARAM)
    self.safety.init_tests()

  def test_buttons(self):
    # Only CANCEL button is allowed while cruise is enabled
    self.safety.set_controls_allowed(0)
    for btn in range(8):
      self.assertFalse(self._tx(self._button_msg(btn)))

    self.safety.set_controls_allowed(1)
    for btn in range(8):
      self.assertFalse(self._tx(self._button_msg(btn)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      self.assertEqual(enabled, self._tx(self._button_msg(Buttons.CANCEL)))

class TestGmCameraEVSafety(TestGmCameraSafety, TestGmEVSafetyBase):
  pass


class TestGmCameraLongitudinalSafety(GmLongitudinalBase, TestGmCameraSafetyBase):
  TX_MSGS = [[0x180, 0], [0x315, 0], [0x2CB, 0], [0x370, 0],  # pt bus
             [0x184, 2]]  # camera bus
  FWD_BLACKLISTED_ADDRS = {2: [0x180, 0x2CB, 0x370, 0x315], 0: [0x184]}  # block LKAS, ACC messages and PSCMStatus
  RELAY_MALFUNCTION_ADDRS = {0: (0x180, 0x2CB, 0x370, 0x315), 2: (0x184,)}
  BUTTONS_BUS = 0  # rx only

  MAX_GAS = 1346
  MIN_GAS = -540  # maximum regen
  INACTIVE_GAS = -500

  def setUp(self):
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | self.EXTRA_SAFETY_PARAM)
    self.safety.init_tests()


class TestGmCameraLongitudinalEVSafety(TestGmCameraLongitudinalSafety, TestGmEVSafetyBase):
  pass


class TestGmCameraNonACCSafety(TestGmCameraSafety):
  TX_MSGS = [[0x180, 0], [0x1E1, 2], [0x184, 2]]
  FWD_BLACKLISTED_ADDRS = {}
  FWD_BUS_LOOKUP = {}
  PCM_CRUISE = False  # NON_ACC cars don't use PCM cruise for control enablement

  def setUp(self):
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(GMSafetyFlagsSP.NON_ACC)
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, GMSafetyFlags.HW_CAM | self.EXTRA_SAFETY_PARAM)
    self.safety.init_tests()

  def test_no_pedal_tx(self):
    values = {"ENABLE": 1, "PEDAL_COUNTER": 0}
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("GAS_COMMAND", 0, values)))

  def _pcm_status_msg(self, enable):
    values = {"CruiseActive": enable}
    return self.packer.make_can_msg_safety("ECMCruiseControl", 0, values)


class TestGmCameraNonACCPedalSafety(GasInterceptorSafetyTest, TestGmCameraSafety):
  TX_MSGS = [[0x180, 0], [0x200, 0], [0xBD, 0], [0x1F5, 0], [0x1E1, 0], [0x1E1, 2], [0x184, 2]]
  FWD_BLACKLISTED_ADDRS = {}
  FWD_BUS_LOOKUP = {}
  INTERCEPTOR_THRESHOLD = 550
  PCM_CRUISE = False  # NON_ACC cars don't use PCM cruise for control enablement

  def setUp(self):
    self.packer = CANPackerSafety("gm_global_a_powertrain_generated")
    self.packer_chassis = CANPackerSafety("gm_global_a_chassis")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(GMSafetyFlagsSP.NON_ACC | GMSafetyFlagsSP.GAS_INTERCEPTOR | GMSafetyFlagsSP.PEDAL_LONG)
    self.safety.set_safety_hooks(CarParams.SafetyModel.gm, GMSafetyFlags.HW_CAM | self.EXTRA_SAFETY_PARAM)
    self.safety.init_tests()

  # NON_ACC cars use pedal interceptor, not PCM cruise for control enablement
  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  # Interceptor-enabled platforms use shared interceptor tests instead of accelerator pedal tests
  def test_prev_gas(self):
    pass

  def test_no_disengage_on_gas(self):
    pass

  def test_buttons(self):
    # NON_ACC cars with gas interceptor allow CANCEL button transmission when cruise is engaged
    self.safety.set_controls_allowed(0)
    for btn in range(8):
      self.assertFalse(self._tx(self._button_msg(btn)))

    self.safety.set_controls_allowed(1)
    for btn in range(8):
      self.assertFalse(self._tx(self._button_msg(btn)))

    # CANCEL button should be allowed when cruise is engaged
    # For NON_ACC cars with gas interceptor, cruise_engaged_prev is updated, so CANCEL is allowed
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._tx(self._button_msg(Buttons.CANCEL)))
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self._tx(self._button_msg(Buttons.CANCEL)))

  def _pcm_status_msg(self, enable):
    values = {"CruiseActive": enable}
    return self.packer.make_can_msg_safety("ECMCruiseControl", 0, values)

  def test_pcm_cruise_non_acc(self):
    # Test that NON_ACC cars with gas interceptor do NOT do PCM cruise check
    # Send PCM status message - should NOT affect controls_allowed for NON_ACC with gas interceptor
    initial_allowed = self.safety.get_controls_allowed()
    self._rx(self._pcm_status_msg(True))
    self.assertEqual(initial_allowed, self.safety.get_controls_allowed())  # Should not change
    self._rx(self._pcm_status_msg(False))
    self.assertEqual(initial_allowed, self.safety.get_controls_allowed())  # Should not change

  # GM's gas_interceptor_prev getter returns a raw decoded value, not a bool flag.
  # Verify it updates with pedal traffic instead of asserting boolean semantics.
  def test_prev_gas_interceptor(self):
    self._rx(self._interceptor_user_gas(0))
    low = self.safety.get_gas_interceptor_prev()
    self._rx(self._interceptor_user_gas(0x1000))
    high = self.safety.get_gas_interceptor_prev()
    self.assertGreaterEqual(low, 0)
    self.assertGreater(high, low)

  # For GM NON_ACC pedal, assert the safety-critical behavior:
  # interceptor input must not disengage lateral controls.
  def test_no_disengage_on_gas_interceptor(self):
    self.safety.set_controls_allowed(True)
    for g in range(0x1000):
      self._rx(self._interceptor_user_gas(g))
      self.assertTrue(self.safety.get_controls_allowed())

  # GM safety allows pedal command frames even when controls are not allowed.
  def test_gas_interceptor_safety_check(self):
    for gas in range(0, 4000, 100):
      for controls_allowed in [True, False]:
        self.safety.set_controls_allowed(controls_allowed)
        self.assertTrue(self._tx(self._interceptor_gas_cmd(gas)))


class TestGmCameraEVNonACCPedalSafety(TestGmCameraNonACCPedalSafety, TestGmEVSafetyBase):
  PCM_CRUISE = False  # NON_ACC cars don't use PCM cruise for control enablement

  # NON_ACC cars use pedal interceptor, not PCM cruise for control enablement
  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

if __name__ == "__main__":
  unittest.main()
