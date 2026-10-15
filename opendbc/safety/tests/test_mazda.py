#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  FWD_BLACKLISTED_ADDRS = {2: [0x243, 0x440]}

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]

  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))


class TestMazdaMainOnMads(unittest.TestCase):
  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()
    self.safety.set_mads_params(True, False, False)

  def tearDown(self):
    self.safety.set_mads_params(False, False, False)

  def _crz_ctrl(self, main_on):
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, {"CRZ_AVAILABLE": main_on})

  def _btns(self, on=False, off=False, bus=0):
    return self.packer.make_can_msg_safety("CRZ_BTNS", bus, {"MODE_Y": on, "MODE_X": off})

  def _standby(self):
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.safety.safety_rx_hook(self._btns())
    self._expire_lateral()

  def _expire_lateral(self):
    self.safety.set_heartbeat_engaged_mads(False)
    for _ in range(3):
      self.safety.mads_heartbeat_engaged_check()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_recorded_on_rearms_standby_without_engaging_cruise(self):
    self._standby()
    # Recorded CX-5 2022 physical ON: MODE_X=MODE_Y=1.
    self.safety.safety_rx_hook(make_msg(0, 0x9d, dat=bytes.fromhex("0061ff0800000000")))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_acc_main_on())
    self.safety.safety_rx_hook(self._btns())
    self.safety.safety_rx_hook(self._crz_ctrl(False))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_fresh_on_gets_a_new_heartbeat_grace_period(self):
    self._standby()
    self.safety.safety_rx_hook(self._btns(on=True))
    for _ in range(2):
      self.safety.mads_heartbeat_engaged_check()
      self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.mads_heartbeat_engaged_check()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_held_on_does_not_rearm_after_heartbeat_exit(self):
    self._standby()
    self.safety.safety_rx_hook(self._btns(on=True))
    self._expire_lateral()
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.safety.safety_rx_hook(self._btns())
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_off_and_camera_bus_do_not_rearm(self):
    self._standby()
    for msg in (self._btns(off=True), self._btns(), self._btns(on=True, bus=2)):
      self.safety.safety_rx_hook(msg)
      self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_requires_main_on_and_mads_enabled(self):
    self.safety.safety_rx_hook(self._crz_ctrl(False))
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._standby()
    self.safety.set_mads_params(False, False, False)
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_pause_on_brake_keeps_request_until_release(self):
    self._standby()
    self.safety.set_mads_params(True, False, True)
    self.safety.safety_rx_hook(self._crz_ctrl(True))
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("PEDALS", 0, {"BRAKE_ON": True}))
    self.safety.safety_rx_hook(self._btns(on=True))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self.safety.get_controls_requested_lateral())
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("PEDALS", 0, {"BRAKE_ON": False}))
    self.assertTrue(self.safety.get_controls_allowed_lateral())


class TestMazdaIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def _msg(self, byte0):
    return make_msg(0, 0x9E, dat=bytes([byte0]) + b"\x00" * 7)

  # 0x9E byte 0 high 3 bits == 6 (0xC0)
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x20))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
