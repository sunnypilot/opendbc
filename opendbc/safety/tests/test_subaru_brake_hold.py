#!/usr/bin/env python3
"""Panda safety tests for the Subaru brake-intercept feature.

Settling window: tx of non-zero ES_Brake is allowed while the rx countdown is
< BRAKE_INTERCEPT_RELEASE_FRAMES (3), i.e. for the first 2 moving Wheel_Speeds frames.
"""
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.subaru.values import SubaruSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.sunnypilot.car.subaru.values_ext import SubaruSafetyFlagsSP

from opendbc.safety.tests.test_subaru import (
  SubaruMsg,
  SUBARU_MAIN_BUS,
  SUBARU_CAM_BUS,
  lkas_tx_msgs,
  TestSubaruSafetyBase,
)

# Brake_Pedal (0x139) and Brake_Status (0x13C) are not in the SubaruMsg enum
MSG_SUBARU_Brake_Pedal  = 0x139
MSG_SUBARU_Brake_Status = 0x13C

BRAKE_INTERCEPT_RELEASE_FRAMES = 3  # must match C #define
SUBARU_BRAKE_HOLD_ACTIVE_FRAMES = 4  # must match C #define (~80ms at 50Hz Wheel_Speeds)


class TestSubaruBrakeIntercept(TestSubaruSafetyBase):
  # Gen1, no SnG, brake_intercept SP param set.
  SAFETY_MODEL = CarParams.SafetyModel.subaru
  FLAGS = 0
  SP_PARAM = SubaruSafetyFlagsSP.BRAKE_INTERCEPT

  # No Brake_Pedal: openpilot never sends it in this no-SnG config, so the relay must forward the
  # car's real Brake_Pedal (0x139) to Eyesight. A leftover allowlist entry statically blocked that
  # relay → Eyesight RX timeout → fault at power-on (hardware-confirmed 2026-05-22).
  TX_MSGS = (
    lkas_tx_msgs(SUBARU_MAIN_BUS)
    + [[SubaruMsg.ES_Brake,      SUBARU_MAIN_BUS]]
    + [[MSG_SUBARU_Brake_Status, SUBARU_CAM_BUS]]
  )

  RELAY_MALFUNCTION_ADDRS = {
    SUBARU_MAIN_BUS: (
      SubaruMsg.ES_LKAS,
      SubaruMsg.ES_DashStatus,
      SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment,
      SubaruMsg.ES_Brake,
    ),
    SUBARU_CAM_BUS: (
      MSG_SUBARU_Brake_Status,
    ),
  }

  # ES_Brake (CAM→MAIN) and Brake_Status (MAIN→CAM) are conditionally blocked only while a hold is
  # actively injected (see TestSubaruBrakeHoldFwd); Brake_Pedal is never blocked.
  FWD_BLACKLISTED_ADDRS = {
    SUBARU_CAM_BUS: [
      SubaruMsg.ES_LKAS,
      SubaruMsg.ES_DashStatus,
      SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment,
    ],
  }

  def setUp(self):
    self.packer = CANPackerSafety("subaru_global_2017_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SP_PARAM)  # must precede set_safety_hooks
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  # ── helpers ────────────────────────────────────────────────────────────────

  def _engage(self):
    # standard engagement: ACC. Long subclass overrides to MADS-only.
    self.safety.set_controls_allowed(True)

  def _es_brake_msg(self, pressure):
    return self.packer.make_can_msg_safety("ES_Brake", SUBARU_MAIN_BUS, {"Brake_Pressure": pressure})

  def _set_standstill(self):
    for _ in range(BRAKE_INTERCEPT_RELEASE_FRAMES + 1):
      self._rx(self._speed_msg(0))

  def _set_moving(self, frames=1):
    for _ in range(frames):
      self._rx(self._speed_msg(10))

  def _exhaust_hysteresis(self):
    self._set_moving(frames=BRAKE_INTERCEPT_RELEASE_FRAMES + 1)

  def test_tx_hook_on_wrong_safety_mode(self):
    # brake-intercept variants share all LKAS TX msgs by design — skip the cross-class overlap check
    raise unittest.SkipTest("Subaru brake-intercept variants share LKAS TX msgs")

  # ── zero pressure always allowed (passthrough) ──────────────────────────────

  def test_es_brake_zero_allowed_at_standstill(self):
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(0)))

  def test_es_brake_zero_allowed_when_moving(self):
    self._exhaust_hysteresis()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(0)))

  def test_es_brake_zero_allowed_when_moving_controls_off(self):
    self._exhaust_hysteresis()
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._es_brake_msg(0)))

  # ── non-zero pressure at standstill: pressure × authority matrix ─────────────

  def test_standstill_pressure_authority(self):
    # (pressure, controls_allowed, controls_lateral, expected) — blocked only when BOTH authorities
    # are off or pressure exceeds max_brake (600)
    for pressure, controls, lateral, allowed in [
      (100, True, False, True),
      (600, True, False, True),   # boundary == max_brake
      (601, True, False, False),  # > max_brake
      (100, False, False, False),  # neither ACC nor MADS
      (100, False, True, True),   # MADS-only
    ]:
      with self.subTest(pressure=pressure, controls=controls, lateral=lateral):
        self._set_standstill()
        self.safety.set_controls_allowed(controls)
        self.safety.set_controls_allowed_lateral(lateral)
        self.assertEqual(allowed, self._tx(self._es_brake_msg(pressure)))

  def test_es_brake_allowed_with_gas_pressed_at_standstill(self):
    # gas is intentionally NOT gated on the AVH path: the same ES_Brake frame carries Eyesight's AEB
    # echo (must never be gas-gated) and braking is the fail-safe direction
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._user_gas_msg(2000))
    self.assertTrue(self._tx(self._es_brake_msg(600)))

  # ── non-zero pressure when moving (hysteresis / settling window) ─────────────

  def test_es_brake_nonzero_blocked_when_moving(self):
    self._set_standstill()
    self._engage()
    self._exhaust_hysteresis()
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_es_brake_nonzero_blocked_when_moving_various_pressures(self):
    self._set_standstill()
    self._engage()
    self._exhaust_hysteresis()
    for pressure in (1, 50, 100, 300, 600):
      with self.subTest(pressure=pressure):
        self.assertFalse(self._tx(self._es_brake_msg(pressure)))

  def test_settling_window_frame_gating(self):
    # countdown < BRAKE_INTERCEPT_RELEASE_FRAMES (3) → settling → allowed; capped at 3 once moving
    for frames, allowed in [(1, True), (2, True), (BRAKE_INTERCEPT_RELEASE_FRAMES, False),
                            (BRAKE_INTERCEPT_RELEASE_FRAMES + 1, False)]:
      with self.subTest(frames=frames):
        self._set_standstill()
        self._engage()
        self._set_moving(frames=frames)
        self.assertEqual(allowed, self._tx(self._es_brake_msg(100)))

  def test_race_a_countdown_resets_on_standstill(self):
    self._set_standstill()
    self._engage()
    self._exhaust_hysteresis()
    self.assertFalse(self._tx(self._es_brake_msg(100)))
    self._set_standstill()  # countdown reset → allowed again
    self.assertTrue(self._tx(self._es_brake_msg(100)))

  def test_race_a_hysteresis_does_not_bypass_controls_allowed(self):
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self._set_moving(frames=1)  # inside settling window
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_race_a_hysteresis_does_not_bypass_max_brake(self):
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self._set_moving(frames=1)  # inside settling window
    self.assertFalse(self._tx(self._es_brake_msg(601)))

  # ── Brake_Status (0x13C) masking tx_hook ────────────────────────────────────

  def _brake_status_msg(self, es_brake_bit):
    return self.packer.make_can_msg_safety("Brake_Status", SUBARU_CAM_BUS, {"ES_Brake": es_brake_bit, "Brake": 0})

  def test_brake_status_allowed_es_brake_cleared(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._brake_status_msg(0)))

  def test_brake_status_blocked_es_brake_set(self):
    # panda must never forward a Brake_Status with ES_Brake=1 to Eyesight
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._brake_status_msg(1)))

  def test_brake_status_allowed_controls_off(self):
    # masking is not gated on controls_allowed — always needed during hold
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._brake_status_msg(0)))

  def test_brake_status_blocked_without_brake_intercept(self):
    self.safety.set_current_safety_param_sp(0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._brake_status_msg(0)))

  def test_no_brake_intercept_es_brake_blocked(self):
    # no brake_intercept SP param → ES_Brake not in allowlist → blocked unconditionally
    self.safety.set_current_safety_param_sp(0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, 0)
    self.safety.init_tests()
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._es_brake_msg(0)))
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_gen2_with_brake_intercept_sp_param_es_brake_blocked(self):
    # gen2 ignores SP brake_intercept — ES_Brake must not be in the allowlist
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.BRAKE_INTERCEPT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, SubaruSafetyFlags.GEN2)
    self.safety.init_tests()
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._es_brake_msg(0)))
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  # ── Conditional forwarding ───────────────────────────────────────────────────
  # ES_Brake (CAM→MAIN) and Brake_Status (MAIN→CAM) are blocked only while a hold is actively
  # injected. Unconditional blocking would starve Eyesight's native ACC braking and trip its
  # ~566ms Cruise_Fault watchdog. Active-hold state is a Wheel_Speeds-paced countdown set on TX.

  def _brake_status_mask_msg(self):
    return self.packer.make_can_msg_safety("Brake_Status", SUBARU_CAM_BUS, {"ES_Brake": 0, "Brake": 0})

  def _tx_hold_pressure(self):
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(400)))

  def _tx_brake_status_mask(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._brake_status_mask_msg()))

  def _pump_wheel_speeds(self, n):
    for _ in range(n):
      self._rx(self._speed_msg(0))

  def test_fwd_es_brake_cam_to_main_allowed_when_idle(self):
    self.assertEqual(SUBARU_MAIN_BUS, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_brake_status_main_to_cam_allowed_when_idle(self):
    self.assertEqual(SUBARU_CAM_BUS, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_es_brake_cam_to_main_blocked_after_hold_tx(self):
    self._tx_hold_pressure()
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_brake_status_main_to_cam_blocked_after_hold_tx(self):
    self._tx_hold_pressure()
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_brake_status_main_to_cam_blocked_after_mask_tx(self):
    self._tx_brake_status_mask()
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_remains_blocked_during_countdown(self):
    self._tx_hold_pressure()
    for k in range(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES - 1):
      self._pump_wheel_speeds(1)
      with self.subTest(after_pumps=k + 1):
        self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
        self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_restored_after_countdown_expires(self):
    self._tx_hold_pressure()
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(SUBARU_MAIN_BUS, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self.assertEqual(SUBARU_CAM_BUS, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_blocked_retriggered_by_subsequent_hold_tx(self):
    self._tx_hold_pressure()
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(SUBARU_MAIN_BUS, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self._set_standstill()
    self.assertTrue(self._tx(self._es_brake_msg(400)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_not_blocked_after_zero_pressure_tx(self):
    # zero-pressure TX must not engage the hold gate
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(0)))
    self.assertEqual(SUBARU_MAIN_BUS, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self.assertEqual(SUBARU_CAM_BUS, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_es_brake_restored_for_aeb_after_hold_release(self):
    # after hold release + countdown decay, Eyesight's AEB ES_Brake (cam→main) must relay again
    self._tx_hold_pressure()
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(SUBARU_MAIN_BUS, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_brake_pedal_main_to_cam_forwarded(self):
    # regression: no-SnG config never sends Brake_Pedal, so the relay forwards the car's real frame
    # MAIN→CAM. A leftover allowlist entry statically blocked it → Eyesight fault (hardware 2026-05-22)
    self.assertEqual(SUBARU_CAM_BUS, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Pedal))

  def test_fwd_brake_pedal_not_gated_during_hold(self):
    # subaru_fwd_hook never gates Brake_Pedal — only ES_Brake / Brake_Status are hold-gated
    self._tx_hold_pressure()
    self.assertEqual(SUBARU_CAM_BUS, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Pedal))


class TestSubaruSnGBrakeIntercept(TestSubaruBrakeIntercept):
  # Gen1, SnG + brake_intercept. SnG re-sends Throttle and Brake_Pedal on the cam bus.
  SP_PARAM = SubaruSafetyFlagsSP.STOP_AND_GO | SubaruSafetyFlagsSP.BRAKE_INTERCEPT

  TX_MSGS = (
    lkas_tx_msgs(SUBARU_MAIN_BUS)
    + [[SubaruMsg.Throttle,        SUBARU_CAM_BUS]]
    + [[MSG_SUBARU_Brake_Pedal,    SUBARU_CAM_BUS]]
    + [[SubaruMsg.ES_Brake,        SUBARU_MAIN_BUS]]
    + [[MSG_SUBARU_Brake_Status,   SUBARU_CAM_BUS]]
  )

  RELAY_MALFUNCTION_ADDRS = {
    SUBARU_MAIN_BUS: (
      SubaruMsg.ES_LKAS,
      SubaruMsg.ES_DashStatus,
      SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment,
      SubaruMsg.ES_Brake,
    ),
    SUBARU_CAM_BUS: (
      SubaruMsg.Throttle,
      MSG_SUBARU_Brake_Pedal,
      MSG_SUBARU_Brake_Status,
    ),
  }

  FWD_BLACKLISTED_ADDRS = {
    SUBARU_CAM_BUS: [
      SubaruMsg.ES_LKAS,
      SubaruMsg.ES_DashStatus,
      SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment,
    ],
    SUBARU_MAIN_BUS: [
      SubaruMsg.Throttle,
      MSG_SUBARU_Brake_Pedal,
    ],
  }

  def test_no_brake_intercept_es_brake_blocked(self):
    # re-init with ONLY SnG (no brake_intercept) → ES_Brake not in allowlist
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.STOP_AND_GO)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, 0)
    self.safety.init_tests()
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._es_brake_msg(0)))
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_gen2_with_brake_intercept_sp_param_es_brake_blocked(self):
    self.safety.set_current_safety_param_sp(
      SubaruSafetyFlagsSP.STOP_AND_GO | SubaruSafetyFlagsSP.BRAKE_INTERCEPT
    )
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, SubaruSafetyFlags.GEN2)
    self.safety.init_tests()
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._es_brake_msg(0)))
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_brake_status_blocked_without_brake_intercept(self):
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.STOP_AND_GO)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._brake_status_msg(0)))

  # SnG sends Brake_Pedal on cam → it IS in the allowlist → relay statically blocked MAIN→CAM.
  # Override the no-SnG regression/invariant, which expect the relay open.
  def test_fwd_brake_pedal_main_to_cam_forwarded(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Pedal))

  def test_fwd_brake_pedal_not_gated_during_hold(self):
    self._tx_hold_pressure()
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Pedal))


class TestSubaruLongBrakeIntercept(TestSubaruBrakeIntercept):
  # Gen1, alpha long enabled, brake_intercept, no SnG. Hardware-failing scenario
  # (route dde08cad3a74cd94|00000012): alpha long ON, MADS active, op long not engaged,
  # AVH wants to hold at standstill. ES_Brake=600 at standstill+MADS must be accepted.
  FLAGS = SubaruSafetyFlags.LONG

  TX_MSGS = (
    lkas_tx_msgs(SUBARU_MAIN_BUS)
    + [[SubaruMsg.ES_Brake,        SUBARU_MAIN_BUS]]
    + [[SubaruMsg.ES_Status,       SUBARU_MAIN_BUS]]
    + [[MSG_SUBARU_Brake_Status,   SUBARU_CAM_BUS]]
  )

  RELAY_MALFUNCTION_ADDRS = {
    SUBARU_MAIN_BUS: (
      SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance, SubaruMsg.ES_Status,
    ),
    SUBARU_CAM_BUS: (
      MSG_SUBARU_Brake_Status,
    ),
  }

  # In long mode op long is the sole sender of ES_Brake/ES_Distance/ES_Status on main, so all three
  # are statically blocked cam→main. Brake_Pedal is not blocked (relay forwards the car's real frame).
  FWD_BLACKLISTED_ADDRS = {
    SUBARU_CAM_BUS: [
      SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance, SubaruMsg.ES_Status,
    ],
  }

  def _engage(self):
    # MADS-only (no ACC): in long mode controls_allowed=True would take the long path, masking the
    # AVH standstill invariant
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)

  def test_avh_hold_pressure_allowed_with_mads_at_standstill(self):
    # alpha long on, op long NOT engaged, MADS active, standstill → AVH may inject ES_Brake=600
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._tx(self._es_brake_msg(600)))

  def test_avh_hold_pressure_blocked_when_moving_without_acc(self):
    # MADS-only: brake injection allowed only at standstill (+ settling); blocked once rolling
    self._exhaust_hysteresis()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self.assertFalse(self._tx(self._es_brake_msg(600)))

  def test_long_path_unchanged_when_acc_engaged(self):
    # adding the AVH union must not narrow the long path: ACC engaged → full ES_Brake range while rolling
    self._exhaust_hysteresis()
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._tx(self._es_brake_msg(100)))

  def test_brake_pressure_above_max_rejected(self):
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)
    self.assertFalse(self._tx(self._es_brake_msg(601)))

  def test_no_controls_no_lateral_blocks_nonzero_brake(self):
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._tx(self._es_brake_msg(100)))
    self.assertTrue(self._tx(self._es_brake_msg(0)))

  def test_brake_status_mask_allowed_with_brake_intercept(self):
    self.assertTrue(self._tx(self._brake_status_msg(0)))

  def test_brake_status_mask_with_es_brake_bit_set_rejected(self):
    self.assertFalse(self._tx(self._brake_status_msg(1)))

  def test_gen2_long_with_brake_intercept_uses_gen2_long_path(self):
    # gen2 has no AVH — Brake_Status must not be in the gen2 long allowlist
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.BRAKE_INTERCEPT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru,
                                  SubaruSafetyFlags.LONG | SubaruSafetyFlags.GEN2)
    self.safety.init_tests()
    self.assertFalse(self._tx(self._brake_status_msg(0)))

  # ── Fwd overrides: in long mode ES_Brake cam→main is ALWAYS statically blocked (op long owns it) ─

  def test_fwd_es_brake_cam_to_main_allowed_when_idle(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_restored_after_countdown_expires(self):
    self._tx_hold_pressure()
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self.assertEqual(SUBARU_CAM_BUS, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_blocked_retriggered_by_subsequent_hold_tx(self):
    self._tx_hold_pressure()
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self._set_standstill()
    self.assertTrue(self._tx(self._es_brake_msg(400)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_not_blocked_after_zero_pressure_tx(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self.assertEqual(SUBARU_CAM_BUS, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_remains_blocked_during_countdown(self):
    self._tx_hold_pressure()
    for k in range(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES - 1):
      self._pump_wheel_speeds(1)
      with self.subTest(after_pumps=k + 1):
        self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
        self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_es_brake_restored_for_aeb_after_hold_release(self):
    # long mode: ES_Brake cam→main stays blocked even after hold release — op long handles AEB
    self._tx_hold_pressure()
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_long_path_brake_does_not_bump_countdown(self):
    # op-long braking (long_valid path) must not bump the hold countdown, else Eyesight's
    # Brake_Status relay starves during sustained ACC braking → ACC faults
    self.safety.set_controls_allowed(True)
    self._set_moving(frames=BRAKE_INTERCEPT_RELEASE_FRAMES + 1)
    self.assertTrue(self._tx(self._es_brake_msg(300)))
    self.assertEqual(SUBARU_CAM_BUS, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_sustained_long_braking_does_not_starve_brake_status_relay(self):
    # 30 consecutive op-long brake TXes must not starve the Brake_Status relay (stay moving so
    # avh_valid=False → long path only)
    self.safety.set_controls_allowed(True)
    self._set_moving(frames=BRAKE_INTERCEPT_RELEASE_FRAMES + 1)
    for _ in range(30):
      self.assertTrue(self._tx(self._es_brake_msg(300)))
      self._rx(self._speed_msg(10))
    self.assertEqual(SUBARU_CAM_BUS, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_safety_reinit_resets_countdown(self):
    # re-calling set_safety_hooks must clear the active-hold countdown immediately
    self._tx_hold_pressure()
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.BRAKE_INTERCEPT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()
    self.assertEqual(SUBARU_CAM_BUS, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))


class TestSubaruLongSnGBrakeIntercept(TestSubaruLongBrakeIntercept):
  # Gen1, alpha long + SnG + brake_intercept — the on-device config for SUBARU_IMPREZA_2020.
  SP_PARAM = SubaruSafetyFlagsSP.STOP_AND_GO | SubaruSafetyFlagsSP.BRAKE_INTERCEPT

  TX_MSGS = (
    lkas_tx_msgs(SUBARU_MAIN_BUS)
    + [[SubaruMsg.ES_Brake,        SUBARU_MAIN_BUS]]
    + [[SubaruMsg.ES_Status,       SUBARU_MAIN_BUS]]
    + [[SubaruMsg.Throttle,        SUBARU_CAM_BUS]]
    + [[MSG_SUBARU_Brake_Pedal,    SUBARU_CAM_BUS]]
    + [[MSG_SUBARU_Brake_Status,   SUBARU_CAM_BUS]]
  )

  RELAY_MALFUNCTION_ADDRS = {
    SUBARU_MAIN_BUS: (
      SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance, SubaruMsg.ES_Status,
    ),
    SUBARU_CAM_BUS: (
      SubaruMsg.Throttle, MSG_SUBARU_Brake_Pedal, MSG_SUBARU_Brake_Status,
    ),
  }

  FWD_BLACKLISTED_ADDRS = {
    SUBARU_CAM_BUS: [
      SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance, SubaruMsg.ES_Status,
    ],
    SUBARU_MAIN_BUS: [
      SubaruMsg.Throttle, MSG_SUBARU_Brake_Pedal,
    ],
  }

  def test_gen2_long_with_brake_intercept_uses_gen2_long_path(self):
    self.safety.set_current_safety_param_sp(
      SubaruSafetyFlagsSP.STOP_AND_GO | SubaruSafetyFlagsSP.BRAKE_INTERCEPT
    )
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru,
                                  SubaruSafetyFlags.LONG | SubaruSafetyFlags.GEN2)
    self.safety.init_tests()
    self.assertFalse(self._tx(self._brake_status_msg(0)))

  # SnG sends Brake_Pedal on cam → in allowlist → relay statically blocked MAIN→CAM
  def test_fwd_brake_pedal_main_to_cam_forwarded(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Pedal))

  def test_fwd_brake_pedal_not_gated_during_hold(self):
    self._tx_hold_pressure()
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Pedal))


if __name__ == "__main__":
  unittest.main()
