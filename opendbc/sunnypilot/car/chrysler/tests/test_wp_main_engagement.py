import unittest

from opendbc.can import CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR, DBC


class ControllerHarness:
  """Exercise the real interface, controller, CAN packer and checksum/counter parser."""
  def __init__(self, candidate=CAR.JEEP_GRAND_CHEROKEE_2019, wp=True):
    fingerprint = gen_empty_fingerprint()
    if wp:
      fingerprint[0][0x4FF] = 4
    cp = CarInterface.get_params(candidate, fingerprint, [], False, False, False)
    cp_sp = CarInterface.get_params_sp(cp, candidate, fingerprint, [], False, False, False)
    self.ci = CarInterface(cp, cp_sp)
    self.cs = self.ci.CS.out = structs.CarState(canValid=True, vEgo=2.0, gearShifter="drive")
    self.ci.CS.lkas_heartbit = dict.fromkeys(("LKAS_DISABLED", "AUTO_HIGH_BEAM", "FORWARD_1", "FORWARD_2", "FORWARD_3"), 0)
    self.cc = structs.CarControl()
    self.cc.actuators.torque = 1.0
    self.cc_sp = structs.CarControlSP()
    self.cc_sp.mads.available = True
    self.parser = CANParser(DBC[candidate][Bus.pt], [("LKAS_COMMAND", 50)], 0)
    self.now = 1_000_000_000
    self.commands = []

  def advance(self, frames):
    commands = []
    for _ in range(frames):
      _, sends = self.ci.apply(self.cc.as_reader(), self.cc_sp, self.now)
      if self.parser.update([self.now, sends]):
        assert self.parser.can_valid, "LKAS message failed checksum/counter validation"
        signals = self.parser.vl["LKAS_COMMAND"]
        commands.append((self.now, signals["LKAS_CONTROL_BIT"], signals["STEERING_TORQUE"], signals["COUNTER"]))
      self.now += 10_000_000
    self.commands.extend(commands)
    return commands

  def main_on(self):
    self.cs.cruiseState.available = True
    self.cc.latActive = True


class TestWpMainEngagement(unittest.TestCase):
  def setUp(self):
    self.h = ControllerHarness()
    # The existing falling-edge guard has expired before MAIN is switched on.
    self.h.advance(300)

  def assert_disabled(self, commands):
    self.assertTrue(commands)
    self.assertTrue(all(request == 0 and torque == 0 for _, request, torque, _ in commands))

  def test_moving_main_on_waits_then_preserves_torque_ramp(self):
    self.h.main_on()
    self.assert_disabled(self.h.advance(70))
    enabled = self.h.advance(100)
    self.assertEqual(enabled[0][1:3], (1, 0))
    self.assertTrue(any(torque > 0 for _, _, torque, _ in enabled))
    torques = [c[2] for c in enabled]
    params = self.h.ci.CC.params
    self.assertLessEqual(max(torques), params.STEER_MAX)
    self.assertTrue(all(0 <= b - a <= params.STEER_DELTA_UP for a, b in zip(torques, torques[1:], strict=False)))

  def test_main_off_removes_request_even_with_stale_lateral_active(self):
    self.h.main_on()
    self.h.advance(100)
    self.h.cs.cruiseState.available = False
    self.assert_disabled(self.h.advance(300))
    self.h.main_on()
    self.assert_disabled(self.h.advance(70))
    self.assertEqual(self.h.advance(2)[0][1], 1)

  def test_main_toggle_during_wait_restarts_the_wait(self):
    self.h.main_on()
    self.assert_disabled(self.h.advance(50))
    self.h.cs.cruiseState.available = False
    self.assert_disabled(self.h.advance(2))
    self.h.main_on()
    self.assert_disabled(self.h.advance(70))
    self.assertEqual(self.h.advance(2)[0][1], 1)

  def test_existing_falling_edge_guard_is_not_shortened(self):
    self.h.main_on()
    self.h.advance(100)
    self.h.cs.cruiseState.available = False
    self.h.cc.latActive = False
    self.assert_disabled(self.h.advance(2))
    self.h.main_on()
    # Existing rule is strictly more than 200 control frames since LKAS fell.
    self.assert_disabled(self.h.advance(200))
    self.assertEqual(self.h.advance(2)[0][1], 1)

  def test_main_off_between_can_transmissions_restarts_wait(self):
    self.h.main_on()
    self.assert_disabled(self.h.advance(51))
    self.h.cs.cruiseState.available = False
    self.assertEqual(self.h.advance(1), [])
    self.h.main_on()
    self.assert_disabled(self.h.advance(70))
    self.assertEqual(self.h.advance(2)[0][1], 1)

  def test_stationary_main_on_can_settle_before_lateral_eligibility(self):
    self.h.cs.cruiseState.available = True
    self.h.cs.vEgo = 0
    self.h.cs.standstill = True
    self.assert_disabled(self.h.advance(400))
    self.h.cs.vEgo = 2.0
    self.h.cs.standstill = False
    self.h.cc.latActive = True
    self.assertEqual(self.h.advance(2)[0][1], 1)

  def test_cancel_and_set_with_mads_do_not_restart_main_timer(self):
    self.h.main_on()
    self.h.advance(100)
    for cruise_enabled in (True, False, True, False):
      self.h.cs.cruiseState.enabled = cruise_enabled
      self.h.cc.enabled = cruise_enabled
      self.h.cc.cruiseControl.cancel = not cruise_enabled
      commands = self.h.advance(20)
      self.assertTrue(all(request == 1 for _, request, _, _ in commands))

  def test_loss_of_lateral_eligibility_never_enables_from_timer_alone(self):
    self.h.main_on()
    self.h.advance(30)
    self.h.cc.latActive = False
    self.assert_disabled(self.h.advance(300))
    self.h.cc.latActive = True
    self.assertEqual(self.h.advance(2)[0][1], 1)
    self.h.cc.latActive = False
    self.assert_disabled(self.h.advance(2))

  def test_invalid_can_and_eps_faults_inhibit_and_require_new_settling(self):
    for field, bad, good in (("canValid", False, True), ("steerFaultTemporary", True, False), ("steerFaultPermanent", True, False)):
      for already_active in (False, True):
        with self.subTest(field=field, already_active=already_active):
          h = ControllerHarness()
          h.advance(300)
          h.main_on()
          h.advance(100 if already_active else 30)
          setattr(h.cs, field, bad)
          self.assert_disabled(h.advance(300))
          setattr(h.cs, field, good)
          self.assert_disabled(h.advance(70))
          self.assertEqual(h.advance(2)[0][1], 1)

  def test_lkas_cadence_and_counters_continue_through_main_transitions(self):
    self.h.main_on()
    self.h.advance(100)
    self.h.cs.cruiseState.available = False
    self.h.advance(250)
    self.h.main_on()
    self.h.advance(100)
    commands = self.h.commands
    self.assertEqual(len(commands), 375)
    for previous, current in zip(commands, commands[1:], strict=False):
      self.assertEqual(current[0] - previous[0], 20_000_000)
      self.assertEqual(current[3], (previous[3] + 1) % 16)

  def test_startup_reenable_protection_is_preserved(self):
    h = ControllerHarness()
    h.main_on()
    self.assert_disabled(h.advance(202))
    self.assertEqual(h.advance(2)[0][1], 1)

  def test_scope_excludes_stock_jeep_and_other_wp_platforms(self):
    for candidate, wp in ((CAR.JEEP_GRAND_CHEROKEE_2019, False), (CAR.JEEP_GRAND_CHEROKEE, True), (CAR.CHRYSLER_PACIFICA_2020, True)):
      with self.subTest(candidate=candidate, wp=wp):
        h = ControllerHarness(candidate, wp)
        h.advance(300)
        h.cs.vEgo = 20.0
        h.main_on()
        self.assertEqual(h.advance(2)[0][1], 1)


if __name__ == "__main__":
  unittest.main()
