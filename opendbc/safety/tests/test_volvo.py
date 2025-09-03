#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda


class TestVolvoSafety(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest):

  TX_MSGS = [[0x127, 0], [0x262, 0], [0x270, 0], [0x246, 2]]
  GAS_PRESSED_THRESHOLD = 10
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {0: [0x262], 2: [0x246]}
  FWD_BLACKLISTED_ADDRS = {2: [0x262], 0: [0x246]}

  VOLVO_MAIN_BUS = 0
  #VOLVO_AUX_BUS = 1
  VOLVO_CAM_BUS = 2

  # Angle control limits
  STEER_ANGLE_MAX = 45  # deg, reasonable limit
  DEG_TO_CAN = 100

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [5., .8, .15]  # windup limit
  ANGLE_RATE_DOWN = [5., 3.5, .4]  # unwind limit

  def setUp(self):
    self.packer = CANPackerPanda("volvo_v60_2015_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volvo, 0)
    self.safety.init_tests()

  def _angle_meas_msg(self, angle: float):
    values = {"SteeringAngleServo": angle}
    return self.packer.make_can_msg_panda("PSCM1", self.VOLVO_MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCStatus": 6 if enable else 0}
    return self.packer.make_can_msg_panda("FSM0", self.VOLVO_CAM_BUS, values)

  def _speed_msg(self, speed: float):
    values = {"VehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_panda("VehicleSpeed1", self.VOLVO_MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePedal": 2 if brake else 0}
    return self.packer.make_can_msg_panda("Brake_Info", self.VOLVO_MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"AccPedal": 10 if gas else 0}
    return self.packer.make_can_msg_panda("AccPedal", self.VOLVO_MAIN_BUS, values)
  
  def _vehicle_moving_msg(self, speed: float):
    values = {"VehicleSpeed": 0 if speed <= self.STANDSTILL_THRESHOLD else 10}
    return self.packer.make_can_msg_panda("VehicleSpeed1", 0, values)

if __name__ == "__main__":
  unittest.main()
