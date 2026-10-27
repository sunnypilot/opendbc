"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC as DBCReader
from opendbc.car import Bus, DT_CTRL, gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.values import DBC, ToyotaSafetyFlags
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP, ToyotaSafetyFlagsSP

SteerControlType = structs.CarParams.SteerControlType

DEG_TO_CAN = 1 / 0.0573


class TestToyotaAngleSteering(unittest.TestCase):
  def _get_params(self, car_name: str, angle_steering: bool, high_output: bool = False):
    fingerprint = gen_empty_fingerprint()
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_params(car_name, fingerprint, [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, car_name, fingerprint, [], False, False, False)
    setup_interfaces(CarInterface, CP, CP_SP, [{"ToyotaAngleSteering": str(int(angle_steering)),
                                               "ToyotaAngleSteeringHighOutput": str(int(high_output))}])
    return CP, CP_SP

  def test_prius_tss2_angle_steering_enabled(self):
    # the stock LKA path is replaced with LTA angle control when the user opts in
    CP, CP_SP = self._get_params("TOYOTA_PRIUS_TSS2", True)
    self.assertEqual(CP.steerControlType, SteerControlType.angle)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.LTA.value)
    self.assertTrue(CP_SP.flags & ToyotaFlagsSP.SP_ANGLE_STEERING)
    self.assertTrue(CP_SP.safetyParam & ToyotaSafetyFlagsSP.SP_ANGLE_STEERING)

    # sunnypilot reads a torque tune as "torque control in use" and drives the torque
    # controller's extension from it, which an angle car doesn't have. Stock Toyota angle
    # platforms skip configure_torque_tune, so an angle car must not keep the torque tune.
    self.assertNotEqual(CP.lateralTuning.which(), 'torque')

    # native LTA actuator delays
    self.assertAlmostEqual(CP.steerActuatorDelay, 0.18)
    self.assertAlmostEqual(CP.steerLimitTimer, 0.8)

  def test_prius_tss2_angle_steering_disabled(self):
    # torque control remains the default
    CP, CP_SP = self._get_params("TOYOTA_PRIUS_TSS2", False)
    self.assertEqual(CP.steerControlType, SteerControlType.torque)
    self.assertFalse(CP.safetyConfigs[0].safetyParam & ToyotaSafetyFlags.LTA.value)
    self.assertFalse(CP_SP.flags & ToyotaFlagsSP.SP_ANGLE_STEERING)
    self.assertFalse(CP_SP.safetyParam & ToyotaSafetyFlagsSP.SP_ANGLE_STEERING)

    # the torque tune stays for the stock LKA path
    self.assertEqual(CP.lateralTuning.which(), 'torque')

  def test_prius_tss2_angle_steering_high_output(self):
    # the wider envelope has its own flag and safety param, only with angle steering on the Prius
    CP, CP_SP = self._get_params("TOYOTA_PRIUS_TSS2", True, True)
    self.assertTrue(CP_SP.flags & ToyotaFlagsSP.SP_ANGLE_STEERING_HIGH_OUTPUT)
    self.assertTrue(CP_SP.safetyParam & ToyotaSafetyFlagsSP.SP_ANGLE_STEERING_HIGH_OUTPUT)

    CP, CP_SP = self._get_params("TOYOTA_PRIUS_TSS2", True, False)
    self.assertFalse(CP_SP.flags & ToyotaFlagsSP.SP_ANGLE_STEERING_HIGH_OUTPUT)
    self.assertFalse(CP_SP.safetyParam & ToyotaSafetyFlagsSP.SP_ANGLE_STEERING_HIGH_OUTPUT)

    CP, CP_SP = self._get_params("TOYOTA_PRIUS_TSS2", False, True)
    self.assertFalse(CP_SP.flags & ToyotaFlagsSP.SP_ANGLE_STEERING_HIGH_OUTPUT)

    CP, CP_SP = self._get_params("TOYOTA_RAV4_TSS2", True, True)
    self.assertFalse(CP_SP.flags & ToyotaFlagsSP.SP_ANGLE_STEERING_HIGH_OUTPUT)

  @staticmethod
  def _get_interface(car_name: str, angle_steering: bool, high_output: bool = False):
    fingerprint = gen_empty_fingerprint()
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_params(car_name, fingerprint, [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, car_name, fingerprint, [], False, False, False)
    setup_interfaces(CarInterface, CP, CP_SP, [{"ToyotaAngleSteering": str(int(angle_steering)),
                                               "ToyotaAngleSteeringHighOutput": str(int(high_output))}])
    return CarInterface(CP, CP_SP)

  @staticmethod
  def _engaged_control(steering_angle_deg: float):
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.actuators.steeringAngleDeg = steering_angle_deg
    return CC.as_reader(), structs.CarControlSP()

  def test_angle_steering_controller(self):
    # the LTA command follows the vehicle model and the tracking error clamp: the 10 deg request
    # is clamped to 6 deg from the (zero) measured angle
    CI = self._get_interface("TOYOTA_PRIUS_TSS2", True)
    CC, CC_SP = self._engaged_control(10.)

    can_sends = []
    for _ in range(10):
      CI.update([])
      _, sends = CI.apply(CC, CC_SP, 0)
      can_sends.extend(sends)

    lta = [m for m in can_sends if m[0] == 0x191]
    self.assertGreater(len(lta), 0)
    _, dat, _ = lta[-1]
    self.assertTrue(dat[0] & 1)  # STEER_REQUEST

    angle_deg = int.from_bytes(dat[1:3], "big", signed=True) / DEG_TO_CAN
    self.assertAlmostEqual(angle_deg, 6., delta=0.2)

  def test_angle_steering_high_output_envelope(self):
    # high-output mode drops the stock tracking error clamp: the same 10 deg request the stock
    # path holds at 6 deg is followed with the higher envelope. The request sets 0x191 B0[1],
    # which is also the LTA counter's LSB: the checksum is recomputed, but the counter LSB is
    # forced, so bench-testing the EPS' counter tolerance is the next step.
    dbc = DBCReader(DBC["TOYOTA_PRIUS_TSS2"][Bus.pt])
    msg = dbc.name_to_msg["STEERING_LTA"]
    checksum_sig = next(s for s in msg.sigs.values() if s.calc_checksum)

    for high_output, expected in ((False, 6.0), (True, 10.0)):
      with self.subTest(high_output=high_output):
        CI = self._get_interface("TOYOTA_PRIUS_TSS2", True, high_output)
        CC, CC_SP = self._engaged_control(10.)

        dat = b""
        for _ in range(10):
          CI.update([])
          _, sends = CI.apply(CC, CC_SP, 0)
          lta = [m for m in sends if m[0] == 0x191]
          if lta:
            dat = lta[-1][1]

        angle_deg = int.from_bytes(dat[1:3], "big", signed=True) / DEG_TO_CAN
        self.assertAlmostEqual(angle_deg, expected, delta=0.2)

        self.assertEqual(bool(dat[0] & 0x02), high_output)
        self.assertEqual(checksum_sig.calc_checksum(msg.address, checksum_sig, bytearray(dat)), dat[7])
        if high_output:
          self.assertEqual(msg.sigs["COUNTER"].get_raw_value(dat) & 1, 1)

  def test_angle_steering_driver_override(self):
    # driver torque above the allowance latches the LTA request off and the angle command follows
    # the measured angle until the driver releases
    CI = self._get_interface("TOYOTA_PRIUS_TSS2", True)
    CC, CC_SP = self._engaged_control(10.)

    packer = CANPacker(DBC["TOYOTA_PRIUS_TSS2"][Bus.pt])
    torque_msg = packer.make_can_msg("STEER_TORQUE_SENSOR", 0, {"STEER_TORQUE_DRIVER": 200., "STEER_TORQUE_EPS": 0.})

    can_sends = []
    now_nanos = 0
    for _ in range(10):
      CI.update([(now_nanos, [torque_msg])])
      _, sends = CI.apply(CC, CC_SP, now_nanos)
      can_sends.extend(sends)
      now_nanos += int(DT_CTRL * 1e9)

    lta = [m for m in can_sends if m[0] == 0x191]
    self.assertGreater(len(lta), 0)
    _, dat, _ = lta[-1]
    self.assertFalse(dat[0] & 1)  # STEER_REQUEST
    self.assertEqual(int.from_bytes(dat[1:3], "big", signed=True), 0)

  def test_angle_steering_only_prius_tss2(self):
    # other TSS2 cars stay on torque control, native LTA cars are unaffected
    for car_name in ("TOYOTA_RAV4_TSS2", "TOYOTA_CAMRY_TSS2", "TOYOTA_COROLLA_TSS2"):
      with self.subTest(car_name=car_name):
        CP, CP_SP = self._get_params(car_name, True)
        self.assertEqual(CP.steerControlType, SteerControlType.torque)
        self.assertFalse(CP_SP.flags & ToyotaFlagsSP.SP_ANGLE_STEERING)

    CP, CP_SP = self._get_params("TOYOTA_RAV4_TSS2_2023", True)
    self.assertEqual(CP.steerControlType, SteerControlType.angle)
    self.assertFalse(CP_SP.flags & ToyotaFlagsSP.SP_ANGLE_STEERING)


if __name__ == "__main__":
  unittest.main()
