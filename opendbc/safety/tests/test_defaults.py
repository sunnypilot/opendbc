#!/usr/bin/env python3
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety.libsafety_py import ffi


class TestDefaultRxHookBase(common.SafetyTest):
  FWD_BUS_LOOKUP = {}

  def test_rx_hook(self):
    # default rx hook allows all msgs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)), f"failed RX {addr=}")


class TestNoOutput(TestDefaultRxHookBase):
  SAFETY_MODEL = CarParams.SafetyModel.noOutput

  TX_MSGS = []


class TestSilent(TestNoOutput):
  """SILENT uses same hooks as NOOUTPUT"""
  SAFETY_MODEL = CarParams.SafetyModel.silent


class TestAllOutput(TestDefaultRxHookBase):
  SAFETY_MODEL = CarParams.SafetyModel.allOutput

  # Allow all messages
  TX_MSGS = [[addr, bus] for addr in common.SafetyTest.SCANNED_ADDRS
             for bus in range(4)]

  def test_spam_can_buses(self):
    # asserts tx allowed for all scanned addrs
    for bus in range(4):
      for addr in self.SCANNED_ADDRS:
        should_tx = [addr, bus] in self.TX_MSGS
        self.assertEqual(should_tx, self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=} {bus=}")

  def test_default_controls_not_allowed(self):
    # controls always allowed
    self.assertTrue(self.safety.get_controls_allowed())

  def test_tx_hook_on_wrong_safety_mode(self):
    # No point, since we allow all messages
    pass


class TestAllOutputPassthrough(TestAllOutput):
  SAFETY_PARAM = 1

  FWD_BLACKLISTED_ADDRS = {}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}


class TestSafetyFramework(common.SafetyTestBase):
  SAFETY_MODEL = CarParams.SafetyModel.noOutput

  def test_unsupported_safety_mode(self):
    self.safety.set_controls_allowed(True)
    self.assertEqual(self.safety.set_safety_hooks(0xFFFF, 0), -1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(common.make_msg(0, 0x123)))


class SafetyRedundancyBase(common.SafetyTestBase):
  SAFETY_MODEL = CarParams.SafetyModel.allOutput

  def _flip_b(self, bit):
    state = common.state_buffer("safety_b")
    state[bit // 8] = bytes([state[bit // 8][0] ^ (1 << (bit % 8))])

  def _assert_fault(self):
    self.assertTrue(self.safety.safety_redundancy_fault)
    self.assertEqual(common.state_buffer("safety_a")[:], common.state_buffer("safety_b")[:])
    self.safety.safety_redundancy_fault = False


class TestSafetyRedundancy(SafetyRedundancyBase):
  def test_rx_decision_mismatch(self):
    # wrong counters in B fail its rx check, CLU11 is a hyundai rx check with a counter and no checksum
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.set_controls_allowed(True)
    for i in range(self.safety.get_rx_checks_len()):
      self._flip_b(ffi.offsetof("SafetyState", "rx_status", i, "wrong_counters") * 8 + 3)
    self.assertTrue(self.safety.safety_rx_hook(common.make_msg(0, 0x4F1, 4)))
    self._assert_fault()

  def test_tx_decision_mismatch(self):
    self.safety.set_controls_allowed(True)
    self._flip_b(ffi.offsetof("SafetyState", "relay_malfunction") * 8)
    self.assertTrue(self.safety.safety_tx_hook(common.make_msg(0, 0x123)))
    self._assert_fault()

  def test_fwd_decision_mismatch(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.allOutput, 1)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x123), 2)
    self._flip_b(ffi.offsetof("SafetyState", "relay_malfunction") * 8)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x123), 2)
    self._assert_fault()

  def test_state_mismatch(self):
    # the tick rewrites safety_rx_checks_invalid in both copies, and a nonzero rx_checks_len makes it read the NULL rx_checks
    rewritten = ffi.offsetof("SafetyCopy", "state") + ffi.offsetof("SafetyState", "safety_rx_checks_invalid")
    rx_checks_len = ffi.offsetof("SafetyCopy", "setup") + ffi.offsetof("SafetySetup", "current_safety_config") + ffi.offsetof("safety_config", "rx_checks_len")
    for byte in range(ffi.sizeof("SafetyCopy")):
      if byte != rewritten and not (rx_checks_len <= byte < rx_checks_len + ffi.sizeof("int")):
        self.safety.set_controls_allowed(True)
        self._flip_b(byte * 8)
        self.safety.safety_tick()
        self._assert_fault()

  def test_same_time(self):
    ford_lat = common.CANPackerSafety("ford_lincoln_base_pt").make_can_msg_safety("LateralMotionControl", 0, {"LatCtl_D_Rq": 1})
    steering = {
      CarParams.SafetyModel.hyundai: (common.make_msg(0, 0x340), lambda s: s.ts_torque_check_last),
      CarParams.SafetyModel.tesla: (common.make_msg(0, 0x488, dat=b"\x40\x00\x20\x00"), lambda s: s.ts_angle_check_last),
      CarParams.SafetyModel.ford: (ford_lat, lambda s: s.curvature_state.ts_check_last),
    }
    for mode, (msg, ts_last) in steering.items():
      with self.subTest(mode=mode):
        self.safety.set_safety_hooks(mode, 0)
        self.safety.set_timer(1000000)
        self.safety.set_timer_step(1)
        self.safety.safety_rx_hook(common.make_msg(0, 0x4F1, 4))
        self.safety.set_controls_allowed(True)
        self.safety.safety_tx_hook(msg)
        self.assertEqual(ts_last(self.safety.safety_a.state), 1000002)


class TestSafetyRedundancySP(SafetyRedundancyBase):
  def test_mads_lateral_decision_mismatch(self):
    # steering is allowed only with controls_allowed_lateral
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.set_mads_params(True, False, False)
    self.safety.set_controls_allowed_lateral(True)
    lkas = common.CANPackerSafety("hyundai_can_generated").make_can_msg_safety("LKAS11", 0, {"CR_Lkas_StrToqReq": 3, "CF_Lkas_ActToi": 1})
    self._flip_b(ffi.offsetof("SafetyState", "controls_allowed_lateral") * 8)
    self.assertTrue(self.safety.safety_tx_hook(lkas))
    self._assert_fault()

  def test_mads_heartbeat_both_copies(self):
    # the panda's entry point, with controls_allowed off so only the MADS check acts
    self.safety.set_controls_allowed(False)
    self.safety.set_mads_params(True, False, False)
    self.safety.set_controls_allowed_lateral(True)
    self.safety.set_heartbeat_engaged_mads(False)
    for _ in range(3):
      self.assertTrue(self.safety.get_controls_allowed_lateral())
      self.safety.safety_heartbeat_check()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.safety_redundancy_fault)

  @staticmethod
  def _padding_masks(ctype):
    # {byte offset: padding bits}, including unused bitfield bits
    used = bytearray(ffi.sizeof(ctype))

    def mark(t, base):
      if t.kind == "struct":
        for _, f in t.fields:
          if f.bitsize >= 0:
            for bit in range(f.bitshift, f.bitshift + f.bitsize):
              used[base + f.offset + bit // 8] |= 1 << (bit % 8)
          else:
            mark(f.type, base + f.offset)
      elif t.kind == "array":
        for i in range(t.length):
          mark(t.item, base + i * ffi.sizeof(t.item))
      else:
        used[base:base + ffi.sizeof(t)] = b"\xff" * ffi.sizeof(t)

    mark(ffi.typeof(ctype), 0)
    return {i: ~u & 0xFF for i, u in enumerate(used) if u != 0xFF}

  def test_mode_change_clears_padding(self):
    # the copies are compared by byte, so padding must match after a mode change
    padding = self._padding_masks("SafetyState")
    self.assertGreater(len(padding), 0)
    state = ffi.offsetof("SafetyCopy", "state")
    for copy, junk in (("safety_a", 0x5A), ("safety_b", 0xA5)):
      buf = common.state_buffer(copy)
      for i, mask in padding.items():
        buf[state + i] = bytes([(buf[state + i][0] & ~mask) | (junk & mask)])

    self.assertEqual(0, self.safety.set_safety_hooks_dirty_stack(CarParams.SafetyModel.hyundai, 0))
    for copy in ("safety_a", "safety_b"):
      buf = common.state_buffer(copy)
      self.assertEqual({}, {i: buf[state + i][0] & mask for i, mask in padding.items() if buf[state + i][0] & mask})
    self.safety.safety_tick()
    self.assertFalse(self.safety.safety_redundancy_fault)

  def test_init_tests_clears_fault(self):
    self.safety.safety_redundancy_fault = True
    self.safety.init_tests()
    self.assertFalse(self.safety.safety_redundancy_fault)


class TestMadsModeChange(common.SafetyTestBase):
  SAFETY_MODEL = CarParams.SafetyModel.noOutput

  def test_mads_flags_kept_across_mode_change(self):
    for disengage_lateral_on_brake, pause_lateral_on_brake in ((True, False), (False, True)):
      with self.subTest(disengage_lateral_on_brake=disengage_lateral_on_brake):
        self.safety.set_mads_params(True, disengage_lateral_on_brake, pause_lateral_on_brake)
        self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
        self.assertTrue(self.safety.get_enable_mads())
        self.assertEqual(disengage_lateral_on_brake, self.safety.get_disengage_lateral_on_brake())
        self.assertEqual(pause_lateral_on_brake, self.safety.get_pause_lateral_on_brake())

        self.safety.tick_mads_state(False, False, False, False, False)
        self.safety.tick_mads_state(False, True, False, False, False)
        self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_mads_state_reset_on_mode_change(self):
    # including into SILENT, which the panda enters when the heartbeat is lost
    for mode in (CarParams.SafetyModel.hyundai, CarParams.SafetyModel.silent):
      with self.subTest(mode=mode):
        self.safety.set_mads_params(True, False, False)
        self.safety.set_heartbeat_engaged_mads(True)
        self.safety.set_controls_allowed(True)
        self.safety.tick_mads_state(True, True, True, False, False)
        self.assertTrue(self.safety.get_controls_allowed_lateral())
        self.safety.set_controls_requested_lateral(True)
        self.safety.set_mads_button_press(1)
        self.safety.mads_set_current_disengage_reason(1)
        for copy in (self.safety.safety_a, self.safety.safety_b):
          copy.state.heartbeat_engaged_mads_mismatches = 2
        self.safety.set_safety_hooks(mode, 0)
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertFalse(self.safety.get_controls_allowed_lateral())
        self.assertFalse(self.safety.get_controls_requested_lateral())
        self.assertEqual(-1, self.safety.get_mads_button_press())
        self.assertFalse(self.safety.get_mads_acc_main())
        self.assertEqual(0, self.safety.mads_get_current_disengage_reason())
        self.assertEqual(0, self.safety.safety_a.state.heartbeat_engaged_mads_mismatches)
        # kept, like heartbeat_engaged
        self.assertTrue(self.safety.safety_a.state.heartbeat_engaged_mads)


class TestHeartbeat(common.SafetyTestBase):
  SAFETY_MODEL = CarParams.SafetyModel.allOutput

  def test_engaged_mismatch(self):
    for engaged in (True, False):
      self.safety.safety_heartbeat(engaged)
      self.safety.set_controls_allowed(True)
      for _ in range(3):
        self.assertTrue(self.safety.get_controls_allowed())
        self.safety.safety_heartbeat_check()
      self.assertEqual(engaged, self.safety.get_controls_allowed())


class TestModeCount(common.SafetyTestBase):
  SAFETY_MODEL = CarParams.SafetyModel.toyota
  SAFETY_PARAM = 73

  def test_relay_malfunction_after_two_ticks(self):
    # the relay malfunction check starts once safety_mode_cnt passes 1
    self.safety.set_safety_hooks(self.SAFETY_MODEL, self.SAFETY_PARAM)
    for _ in range(2):
      self.safety.safety_rx_hook(common.make_msg(0, 0x2E4, 8))
      self.assertFalse(self.safety.get_relay_malfunction())
      self.safety.safety_tick()
    self.safety.safety_rx_hook(common.make_msg(0, 0x2E4, 8))
    self.assertTrue(self.safety.get_relay_malfunction())


if __name__ == "__main__":
  unittest.main()
