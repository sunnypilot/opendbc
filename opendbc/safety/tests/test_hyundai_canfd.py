#!/usr/bin/env python3
from opendbc.testing import parameterized_class
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase

# All combinations of radar/camera-SCC and gas/hybrid/EV cars
ALL_GAS_EV_HYBRID_COMBOS = [
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]


class TestHyundaiCanfdBase(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [270]

  MAX_RT_DELTA = 112

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  def _torque_driver_msg(self, torque):
    values = {"MDPS_StrTqSnsrVal": torque}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"StrTqReqVal": torque, "ActToiSta": steer_req}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed):
    values = {f"WHL_Spd{pos}Val": speed * 0.03125 for pos in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_safety(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", bus, values)

  def _acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.PT_BUS, values)

  def _main_cruise_button_msg(self, enabled):
    return self._button_msg(0, enabled)


class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdBase):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0)}  # LFA, LFAHDA_CLUSTER
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0]}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.SAFETY_PARAM)
    self.safety.init_tests()


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass


class TestHyundaiCanfdLFACameraSync(TestHyundaiCanfdLFASteeringBase):
  TX_MSGS = [[0x7FF, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x1E0,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x12A]}
  MAX_TORQUE_LOOKUP = [0], [384]
  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 7

  GAS_MSG = ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL")
  SAFETY_PARAM = HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_LFA_CAMERA_SYNC

  @staticmethod
  def _command(torque, request, ui_active=0, magic=0xA5, reserved=0):
    dat = int(torque).to_bytes(2, byteorder="little", signed=True) + bytes([request, magic, ui_active, reserved, 0, 0])
    return libsafety_py.make_CANPacket(0x7FF, 0, dat)

  def _torque_cmd_msg(self, torque, steer_req=1):
    return self._command(torque, steer_req)

  def _camera_lfa(self, torque=0, request=1, counter=0, **kwargs):
    values = {"COUNTER": counter, "ActToiSta": request, "StrTqReqVal": torque, **kwargs}
    return self.packer.make_can_msg_safety("LFA", 2, values)

  def _camera_1e0(self, counter=0, **kwargs):
    return self.packer.make_can_msg_safety("LFAHDA_CLUSTER", 2, {"COUNTER": counter, **kwargs})

  def _camera_161(self, counter=0, **kwargs):
    return self.packer.make_can_msg_safety("CCNC_0x161", 2, {"COUNTER": counter, **kwargs})

  def _camera_162(self, counter=0, **kwargs):
    return self.packer.make_can_msg_safety("CCNC_0x162", 2, {"COUNTER": counter, **kwargs})

  def _mdps(self, counter=0, **kwargs):
    return self.packer.make_can_msg_safety("MDPS", 0, {"COUNTER": counter, **kwargs})

  @staticmethod
  def _data(msg):
    return bytes(msg[0].data[0:16])

  def _modify(self, msg):
    self.safety.safety_fwd_modify(msg)
    return self._data(msg)

  def _modify_len(self, msg, length):
    self.safety.safety_fwd_modify(msg)
    return bytes(msg[0].data[0:length])

  def test_lfa_forwarding_direction(self):
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x12A), 0)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x12A), -1)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x1E0), 0)

  def test_virtual_command_is_consumed(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._command(0, 1)))
    self.assertTrue(self.safety.safety_tx_consumed)

    self.assertFalse(self._tx(self._command(0, 1, magic=0)))
    self.assertTrue(self.safety.safety_tx_consumed)

  def test_disengaged_always_suppresses_stock_steering(self):
    msg = self._camera_lfa(37, 1, 41, Damping_Gain=47, NEW_SIGNAL_1=5, NEW_SIGNAL_7=0xA6)
    modified = self._modify(msg)
    expected = self._camera_lfa(0, 0, 41, Damping_Gain=100, NEW_SIGNAL_1=5, NEW_SIGNAL_7=0xA6)
    self.assertEqual(modified, self._data(expected))

  def test_command_owns_steering_with_or_without_camera_lanes(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._command(2, 1)))

    for camera_request, damping in ((0, 10), (1, 47)):
      with self.subTest(camera_request=camera_request):
        msg = self._camera_lfa(91, camera_request, 7, Damping_Gain=47, NEW_SIGNAL_1=5, NEW_SIGNAL_7=0xA6)
        modified = self._modify(msg)
        expected = self._camera_lfa(2, 1, 7, Damping_Gain=damping, NEW_SIGNAL_1=5, NEW_SIGNAL_7=0xA6)
        self.assertEqual(modified, self._data(expected))

  def test_camera_companion_messages_pass_unchanged(self):
    msg = self._camera_1e0(5, LFA_ICON=1)
    original = self._data(msg)
    self.assertEqual(self._modify(msg), original)

  def test_camera_ui_tracks_openpilot_without_touching_faults(self):
    self.safety.set_controls_allowed(True)
    for ui_active in (False, True):
      with self.subTest(ui_active=ui_active):
        self.assertTrue(self._tx(self._command(0, 0, ui_active)))
        msg = self._camera_161(9, LFA_ICON=3, CENTERLINE=2, ALERTS_2=5, LANELINE_LEFT=6)
        expected = self._camera_161(9, LFA_ICON=2 if ui_active else 0, CENTERLINE=1 if ui_active else 0,
                                    ALERTS_2=5, LANELINE_LEFT=6)
        self.assertEqual(self._modify_len(msg, 32), bytes(expected[0].data[0:32]))

    faults = self._camera_162(11, FAULT_LSS=1, FAULT_LFA=2, FAULT_DAS=3)
    original = bytes(faults[0].data[0:32])
    self.assertEqual(self._modify_len(faults, 32), original)

  def test_camera_hands_off_warning_is_suppressed_while_active(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._command(0, 0, ui_active=1)))

    for alerts_2, alerts_5, sounds_2 in ((1, 0, 0), (2, 0, 3), (4, 1, 6), (0, 1, 0)):
      with self.subTest(alerts_2=alerts_2, alerts_5=alerts_5, sounds_2=sounds_2):
        msg = self._camera_161(9, ALERTS_2=alerts_2, ALERTS_5=alerts_5, SOUNDS_2=sounds_2, ALERTS_3=7)
        expected = self._camera_161(9, ALERTS_2=0, ALERTS_5=0, SOUNDS_2=0, ALERTS_3=7, LFA_ICON=2, CENTERLINE=1)
        self.assertEqual(self._modify_len(msg, 32), bytes(expected[0].data[0:32]))

    # Other warnings, and the hands-off values while openpilot is inactive, pass through.
    unrelated = self._camera_161(10, ALERTS_2=5, ALERTS_5=3, SOUNDS_2=4)
    expected = self._camera_161(10, ALERTS_2=5, ALERTS_5=3, SOUNDS_2=4, LFA_ICON=2, CENTERLINE=1)
    self.assertEqual(self._modify_len(unrelated, 32), bytes(expected[0].data[0:32]))

    self.assertTrue(self._tx(self._command(0, 0, ui_active=0)))
    inactive = self._camera_161(11, ALERTS_2=4, ALERTS_5=1, SOUNDS_2=6)
    expected = self._camera_161(11, ALERTS_2=4, ALERTS_5=1, SOUNDS_2=6)
    self.assertEqual(self._modify_len(inactive, 32), bytes(expected[0].data[0:32]))

  def test_active_damping_uses_speed_schedule_without_lanes(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._command(2, 1)))

    for speed_kph, expected_damping in ((0, 10), (30, 10), (40, 25), (50, 35), (60, 40),
                                        (80, 60), (100, 80), (104, 81), (108, 82), (130, 82)):
      with self.subTest(speed_kph=speed_kph):
        # Let the measured 0.1 first-order filter settle at each speed.
        for _ in range(100):
          speed = self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS,
                                                  {f"WHL_Spd{pos}Val": speed_kph for pos in ("FL", "FR", "RL", "RR")})
          self._rx(speed)
        modified = self._modify(self._camera_lfa(0, 0, counter=2, Damping_Gain=100))
        self.assertEqual(modified[13], expected_damping)

  def test_damping_only_updates_on_even_lfa_counters(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._command(2, 1)))

    for speed_kph in (40, 80):
      for _ in range(100):
        speed = self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS,
                                                {f"WHL_Spd{pos}Val": speed_kph for pos in ("FL", "FR", "RL", "RR")})
        self._rx(speed)
      counter = 2 if speed_kph == 40 else 3
      modified = self._modify(self._camera_lfa(0, 0, counter=counter, Damping_Gain=100))
      self.assertEqual(modified[13], 25)

  def test_camera_sees_acknowledgement_matching_its_request(self):
    for camera_request in (0, 1):
      with self.subTest(camera_request=camera_request):
        self._modify(self._camera_lfa(0, camera_request))
        msg = self._mdps(19, MDPS_LkaToiActvSta=1 - camera_request, MDPS_LkaPlgInSta=1)
        modified = self._modify_len(msg, 24)
        expected = self._mdps(19, MDPS_LkaToiActvSta=camera_request, MDPS_LkaPlgInSta=1)
        self.assertEqual(modified, bytes(expected[0].data[0:24]))

  def test_stale_disengaged_and_unsafe_commands_fail_closed(self):
    for reason in ("stale", "disengaged", "unsafe"):
      with self.subTest(reason=reason):
        self._reset_safety_hooks()
        self.safety.set_timer(0)
        self.safety.set_controls_allowed(True)

        allowed = self._tx(self._command(self.MAX_RATE_UP + 1 if reason == "unsafe" else 2, 1))
        self.assertEqual(allowed, reason != "unsafe")
        if reason == "stale":
          self.safety.set_timer(50001)
        elif reason == "disengaged":
          self.safety.set_controls_allowed(False)

        msg = self._camera_lfa(91, 1, 10, Damping_Gain=47)
        expected = self._camera_lfa(0, 0, 10, Damping_Gain=100)
        self.assertEqual(self._modify(msg), self._data(expected))


class TestHyundaiCanfdLFASteeringAltButtonsBase(TestHyundaiCanfdLFASteeringBase):

  SAFETY_PARAM: int

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _acc_cancel_msg(self, cancel, accel=0):
    values = {"ACCMode": 4 if cancel else 0, "aReqRaw": accel, "aReqValue": accel}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_button_sends(self):
    """
      No button send allowed with alt buttons.
    """
    for enabled in (True, False):
      for btn in range(8):
        self.safety.set_controls_allowed(enabled)
        self.assertFalse(self._tx(self._button_msg(btn)))

  def test_acc_cancel(self):
    # FIXME: the CANFD_ALT_BUTTONS cars are the only ones that use SCC_CONTROL to cancel, why can't we use buttons?
    for enabled in (True, False):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._acc_cancel_msg(True)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel=1)))
      self.assertFalse(self._tx(self._acc_cancel_msg(False)))


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringAltButtons(TestHyundaiCanfdLFASteeringAltButtonsBase):
  pass


class TestHyundaiCanfdLKASteeringEV(TestHyundaiCanfdBase):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # LKAS, CAM_0x2A4
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()


# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEV(TestHyundaiCanfdBase):

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}  # LKAS_ALT, CAM_0x362
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringLongEV(HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4), 1: (0x1a0,)}  # LKAS, CAM_0x2A4, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG |
                                 HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)


# Tests longitudinal for ICE, hybrid, EV cars with LFA steering
class TestHyundaiCanfdLFASteeringLongBase(HyundaiLongitudinalBase, TestHyundaiCanfdLFASteeringBase):

  FWD_BLACKLISTED_ADDRS = {2: [0x12a, 0x1e0, 0x1a0, 0x160]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x1a0, 0x160)}  # LFA, LFAHDA_CLUSTER, SCC_CONTROL, ADRV_0x160

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongBase":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  # no knockout
  def test_tester_present_allowed(self):
    pass


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLong(TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      cls.safety = None
      raise unittest.SkipTest


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongAltButtons(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongAltButtons":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def test_acc_cancel(self):
    # Alt buttons does not use SCC_CONTROL to cancel if longitudinal
    pass


if __name__ == "__main__":
  unittest.main()
