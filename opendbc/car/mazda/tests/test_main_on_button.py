"""A physical ON press can request MADS through the existing LKAS event."""
import unittest

from opendbc.testing import parameterized

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.sunnypilot.car.mazda.mads import MADS_ON_STANDBY_FRAMES

ButtonType = structs.CarState.ButtonEvent.Type
ON = bytes.fromhex('0061ff0800000000')
OFF = bytes.fromhex('0101fdf000000000')


class StandbyRig:
  def __init__(self):
    fingerprint = gen_empty_fingerprint()
    cp = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=False, is_release=False, docs=False)
    cp_sp = CarInterface.get_params_sp(cp, CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
    self.ci = CarInterface(cp, cp_sp)
    self.pk = CANPacker('mazda_2017')
    self.frame = 0
    self.cc = structs.CarControl()
    self.cc_sp = structs.CarControlSP()
    self.cc_sp.mads.available = True

  def step(self, buttons=None, raw=None, feedback=True, available=True, cruise=False, brake=False):
    if feedback:
      self.ci.CS.update_mads_feedback(self.cc, self.cc_sp)
    buttons = {'BIT1': 1, **(buttons or {})}
    frames = [
      self.pk.make_can_msg('CRZ_BTNS', 0, buttons) if raw is None else (0x9d, raw, 0),
      self.pk.make_can_msg('PEDALS', 0, {'ACC_OFF': available and not cruise, 'ACC_ACTIVE': cruise, 'BRAKE_ON': brake}),
      self.pk.make_can_msg('CRZ_CTRL', 0, {'CRZ_AVAILABLE': available, 'CRZ_ACTIVE': cruise}),
    ]
    self.frame += 1
    cs, _ = self.ci.update([(self.frame * 10_000_000, frames)])
    return cs

  def settle(self, **kwargs):
    for _ in range(MADS_ON_STANDBY_FRAMES + 1):
      cs = self.step(**kwargs)
      assert not lkas_events(cs)
    return cs


def lkas_events(cs):
  return [be.pressed for be in cs.buttonEvents if be.type == ButtonType.lkas]


class TestMazdaMainOnButton(unittest.TestCase):
  def test_recorded_on_requests_lkas_only_from_disabled_standby(self):
    rig = StandbyRig()
    rig.settle()
    cs = rig.step(raw=ON)
    assert cs.cruiseState.available and (not cs.cruiseState.enabled)
    assert lkas_events(cs) == [True]
    assert lkas_events(rig.step(raw=ON)) == []
    assert lkas_events(rig.step()) == [False]

  def test_feedback_reaches_carstate_through_controller(self):
    rig = StandbyRig()
    rig.step()
    rig.cc_sp.mads.enabled = True
    rig.ci.apply(rig.cc.as_reader(), rig.cc_sp, 10000000)
    assert rig.ci.CS.mads_available
    assert rig.ci.CS.mads_enabled
    assert rig.ci.CS.mads_feedback_age == 0
    rig.cc_sp.mads.enabled = False
    rig.ci.apply(rig.cc.as_reader(), rig.cc_sp, 20000000)
    assert not rig.ci.CS.mads_enabled

  @parameterized('enabled,active,lat_active', [(True, True, True), (True, False, False), (False, True, False), (False, False, True)])
  def test_enabled_paused_or_active_feedback_prevents_toggle_off(self, enabled, active, lat_active):
    rig = StandbyRig()
    rig.cc_sp.mads.enabled = enabled
    rig.cc_sp.mads.active = active
    rig.cc.latActive = lat_active
    rig.settle()
    assert lkas_events(rig.step(raw=ON)) == []

  def test_new_enabled_feedback_blocks_even_after_standby_settled(self):
    rig = StandbyRig()
    rig.settle()
    rig.cc_sp.mads.enabled = True
    assert lkas_events(rig.step(raw=ON)) == []

  def test_missing_disabled_or_stale_feedback_cannot_request(self):
    rig = StandbyRig()
    rig.settle(feedback=False)
    assert lkas_events(rig.step(raw=ON, feedback=False)) == []
    rig.step()
    rig.cc_sp.mads.available = False
    rig.settle()
    assert lkas_events(rig.step(raw=ON)) == []
    rig.step()
    rig.cc_sp.mads.available = True
    rig.settle()
    rig.settle(feedback=False)
    assert lkas_events(rig.step(raw=ON, feedback=False)) == []

  def test_feedback_recovery_after_offroad_waits_for_a_fresh_press(self):
    rig = StandbyRig()
    rig.settle(feedback=False)
    assert lkas_events(rig.step(raw=ON, feedback=False)) == []
    for _ in range(MADS_ON_STANDBY_FRAMES + 1):
      assert lkas_events(rig.step(raw=ON)) == []
    rig.step()
    assert lkas_events(rig.step(raw=ON)) == [True]

  def test_rapid_repress_is_suppressed_while_feedback_catches_up(self):
    rig = StandbyRig()
    rig.settle()
    assert lkas_events(rig.step(raw=ON)) == [True]
    assert lkas_events(rig.step()) == [False]
    assert lkas_events(rig.step(raw=ON)) == []
    rig.cc_sp.mads.enabled = True
    rig.step()
    rig.settle()
    assert lkas_events(rig.step(raw=ON)) == []

  def test_rejected_request_can_be_retried_after_settling(self):
    rig = StandbyRig()
    rig.settle()
    assert lkas_events(rig.step(raw=ON)) == [True]
    assert lkas_events(rig.step()) == [False]
    rig.settle()
    assert lkas_events(rig.step(raw=ON)) == [True]

  @parameterized('available,cruise', [(False, False), (True, True)])
  def test_requires_real_standby(self, available, cruise):
    rig = StandbyRig()
    rig.settle(available=available, cruise=cruise)
    assert lkas_events(rig.step(raw=ON, available=available, cruise=cruise)) == []

  def test_main_rising_edge_uses_the_existing_mads_path(self):
    rig = StandbyRig()
    rig.settle(available=False)
    assert lkas_events(rig.step(raw=ON)) == []

  @parameterized('buttons', [{'MODE_X': 1}, {'CAN_OFF': 1}, {'RES': 1}, {'SET_P': 1}, {'SET_M': 1}])
  def test_off_and_speed_buttons_never_request_lkas(self, buttons):
    rig = StandbyRig()
    rig.settle()
    assert lkas_events(rig.step(buttons=buttons)) == []
    assert lkas_events(rig.step(raw=OFF)) == []

  @parameterized('button', ['RES', 'SET_P', 'SET_M', 'CAN_OFF'])
  def test_recent_cruise_activity_restarts_standby_guard(self, button):
    rig = StandbyRig()
    rig.settle()
    rig.step(buttons={button: 1})
    rig.step()
    assert lkas_events(rig.step(raw=ON)) == []

  @parameterized('mode', [ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE, ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE])
  def test_restricted_brake_modes_require_release_and_new_press(self, mode):
    rig = StandbyRig()
    rig.ci.CP.alternativeExperience |= mode
    rig.settle(brake=True)
    assert lkas_events(rig.step(raw=ON, brake=True)) == []
    rig.step()
    rig.settle()
    assert lkas_events(rig.step(raw=ON)) == [True]

  def test_remain_active_mode_allows_on_with_brake_held(self):
    rig = StandbyRig()
    rig.settle(brake=True)
    assert lkas_events(rig.step(raw=ON, brake=True)) == [True]


if __name__ == '__main__':
  unittest.main()
