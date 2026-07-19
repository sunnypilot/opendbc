import unittest
from unittest.mock import MagicMock, patch

from opendbc.can import CANParser
from opendbc.car import structs, Bus
from opendbc.car.subaru.values import DBC, CAR, SubaruFlags, CarControllerParams
from opendbc.car.subaru.carcontroller import CarController
from opendbc.car.interfaces import GearShifter

_DUMMY_MSG = ("ES_Brake", bytes(8), 0)
_DUMMY_BS_MSG = ("Brake_Status", bytes(8), 2)
_DBC_NAMES = DBC[CAR.SUBARU_IMPREZA_2020.value]


def make_CP(brake_hold: bool = True, long_control: bool = False) -> structs.CarParams:
  CP = structs.CarParams()
  CP.carFingerprint = CAR.SUBARU_IMPREZA_2020.value
  CP.openpilotLongitudinalControl = long_control
  if brake_hold:
    CP.flags = SubaruFlags.BRAKE_HOLD.value | SubaruFlags.STEER_RATE_LIMITED.value
  else:
    CP.flags = SubaruFlags.STEER_RATE_LIMITED.value
  return CP


def make_CC(lat_active: bool = False, long_active: bool = False, enabled: bool = False, cancel: bool = False):
  # capnp reader so actuators.as_builder() works in update()
  cc_b = structs.CarControl()
  cc_b.enabled = enabled
  cc_b.latActive = lat_active
  cc_b.longActive = long_active
  cc_b.cruiseControl.cancel = cancel
  return cc_b.as_reader()


def make_CC_SP(mads_enabled: bool = True, mads_active: bool = True) -> structs.CarControlSP:
  CC_SP = structs.CarControlSP()
  CC_SP.mads.enabled = mads_enabled
  CC_SP.mads.active = mads_active
  return CC_SP


def make_CS(
  vEgoRaw: float = 0.0,
  brakePressed: bool = False,
  gasPressed: bool = False,
  standstill: bool = True,
  gear: GearShifter = GearShifter.drive,
  aeb_status: int = 0,
  es_brake_msg=None,
  cruise_enabled: bool = False,
  cruise_available: bool = False,
):
  CS = MagicMock()
  CS.out = MagicMock()
  CS.out.vEgoRaw = vEgoRaw
  CS.out.brakePressed = brakePressed
  CS.out.gasPressed = gasPressed
  CS.out.standstill = standstill
  CS.out.gearShifter = gear
  CS.out.steeringTorque = 0
  CS.out.steeringRateDeg = 0
  CS.out.cruiseState = MagicMock()
  CS.out.cruiseState.enabled = cruise_enabled
  CS.out.cruiseState.available = cruise_available
  CS.es_distance_msg = {"COUNTER": 0, "Cruise_Cancel": False, "Cruise_Throttle": 0,
                         "Close_Distance": 0.0}
  CS.es_dashstatus_msg = {}
  CS.es_lkas_state_msg = {}
  CS.es_infotainment_msg = {}
  CS.es_status_msg = {}
  if es_brake_msg is None:
    CS.es_brake_msg = {
      "AEB_Status": aeb_status, "CHECKSUM": 0, "Signal1": 0,
      "Brake_Pressure": 0, "Cruise_Brake_Lights": 0,
      "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 0,
      "Cruise_Activated": 0, "Signal3": 0,
    }
  else:
    CS.es_brake_msg = es_brake_msg
  CS.brake_status_msg = {
    "CHECKSUM": 0, "COUNTER": 0, "Signal1": 0, "ES_Brake": 0,
    "Signal2": 0, "Brake": 0, "Signal3": 0,
  }
  CS.cruise_button = 0
  CS.throttle_msg = {}
  CS.brake_pedal_msg = {}
  return CS


def make_ctrl(brake_hold: bool = True, long_control: bool = False) -> CarController:
  CP = make_CP(brake_hold=brake_hold, long_control=long_control)
  CP_SP = structs.CarParamsSP()  # no SnG flags → SnGCarController disabled
  return CarController(_DBC_NAMES, CP, CP_SP)


# patch the unrelated send fns that choke on the mock CS data
_STEERING_PATCH = patch("opendbc.car.subaru.subarucan.create_steering_control", return_value=_DUMMY_MSG)
_DASHSTATUS_PATCH = patch("opendbc.car.subaru.subarucan.create_es_dashstatus", return_value=_DUMMY_MSG)
_LKAS_STATE_PATCH = patch("opendbc.car.subaru.subarucan.create_es_lkas_state", return_value=_DUMMY_MSG)
_BRAKE_HOLD_PATCH = "opendbc.car.subaru.subarucan.create_es_brake_hold"
_BRAKE_STATUS_HOLD_PATCH = "opendbc.sunnypilot.car.subaru.subarucan_ext.create_brake_status_hold"


def run_update(ctrl, CC=None, CC_SP=None, CS=None):
  CC = CC or make_CC()
  CC_SP = CC_SP or make_CC_SP()
  CS = CS or make_CS()
  with _STEERING_PATCH, _DASHSTATUS_PATCH, _LKAS_STATE_PATCH, \
       patch(_BRAKE_HOLD_PATCH, return_value=_DUMMY_MSG) as mock_bh, \
       patch(_BRAKE_STATUS_HOLD_PATCH, return_value=_DUMMY_BS_MSG):
    ctrl.update(CC, CC_SP, CS, 0)
  return mock_bh


def run_update_capture_both(ctrl, CC=None, CC_SP=None, CS=None):
  CC = CC or make_CC()
  CC_SP = CC_SP or make_CC_SP()
  CS = CS or make_CS()
  with _STEERING_PATCH, _DASHSTATUS_PATCH, _LKAS_STATE_PATCH, \
       patch(_BRAKE_HOLD_PATCH, return_value=_DUMMY_MSG) as mock_bh, \
       patch(_BRAKE_STATUS_HOLD_PATCH, return_value=_DUMMY_BS_MSG) as mock_bsh:
    ctrl.update(CC, CC_SP, CS, 0)
  return mock_bh, mock_bsh


class TestBrakeHoldController(unittest.TestCase):

  def _get_brake_value(self, mock_bh):
    self.assertTrue(mock_bh.called, "create_es_brake_hold was not called")
    return mock_bh.call_args[0][4]  # (packer, frame, es_brake_msg, long_enabled, brake_value)

  def _prime(self, ctrl):
    ctrl.frame = 1  # odd frame: priming runs but frame%5 != 0 (isolates priming from hold/reset)
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True), CS=make_CS(vEgoRaw=0.5, brakePressed=True))
    self.assertTrue(ctrl._brake_hold_primed, "precondition: must be primed")

  def test_primed_false_on_init(self):
    self.assertFalse(make_ctrl()._brake_hold_primed)

  def test_priming_conditions(self):
    # priming requires mads.enabled AND vEgoRaw<1.5 AND brakePressed AND gear not park/reverse
    for mads_enabled, vego, brake, gear, expected in [
      (False, 0.5, True, GearShifter.drive, False),  # mads disabled
      (True, 0.5, False, GearShifter.drive, False),  # brake not pressed
      (True, 2.0, True, GearShifter.drive, False),  # speed >= 1.5
      (True, 0.5, True, GearShifter.park, False),  # park
      (True, 0.5, True, GearShifter.reverse, False),  # reverse
      (True, 0.5, True, GearShifter.drive, True),  # all met → primed
    ]:
      with self.subTest(mads_enabled=mads_enabled, vego=vego, brake=brake, gear=gear):
        ctrl = make_ctrl()
        ctrl.frame = 1
        run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=mads_enabled),
                   CS=make_CS(vEgoRaw=vego, brakePressed=brake, gear=gear))
        self.assertIs(ctrl._brake_hold_primed, expected)

  def test_holding_conditions(self):
    for primed, standstill, brake, gas, gear, holds in [
      (False, True, False, False, GearShifter.drive, False),  # not primed
      (True, False, False, False, GearShifter.drive, False),  # not standstill — ACC may be braking
      (True, True, True, False, GearShifter.drive, True),   # seamless hold, foot still on pedal
      (True, True, False, True,  GearShifter.drive, False),  # gas pressed
      (True, True, False, False, GearShifter.park, False),  # park
      (True, True, False, False, GearShifter.drive, True),   # all met
    ]:
      with self.subTest(primed=primed, standstill=standstill, brake=brake, gas=gas, gear=gear):
        ctrl = make_ctrl()
        ctrl.frame = 0  # frame%5 == 0: hold branch runs
        ctrl._brake_hold_primed = primed
        mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                             CS=make_CS(standstill=standstill, brakePressed=brake, gasPressed=gas, gear=gear))
        if holds:
          self.assertEqual(self._get_brake_value(mock_bh), CarControllerParams.BRAKE_HOLD_PRESSURE)
        else:
          self.assertFalse(mock_bh.called)

  def test_aeb_vs_hold_brake_value(self):
    for primed, standstill, vego, aeb_status, eyesight_pressure, expected in [
      (True, True, 0.0, 8, 450, 450),  # AEB overrides hold → echo Eyesight pressure
      (True, True, 0.0, 0, 0, CarControllerParams.BRAKE_HOLD_PRESSURE),  # no AEB, holding → hold pressure
      (False, False, 8.0, 4, 600, 600),  # AEB while moving, not holding → passthrough
    ]:
      with self.subTest(aeb_status=aeb_status, eyesight_pressure=eyesight_pressure):
        ctrl = make_ctrl()
        ctrl.frame = 0
        ctrl._brake_hold_primed = primed
        es_msg = {"AEB_Status": aeb_status, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": eyesight_pressure,
                  "Cruise_Brake_Lights": 0, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 0,
                  "Cruise_Activated": 0, "Signal3": 0}
        mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                             CS=make_CS(standstill=standstill, vEgoRaw=vego, brakePressed=False,
                                        gasPressed=False, es_brake_msg=es_msg))
        self.assertEqual(self._get_brake_value(mock_bh), expected)

  def test_latch_reset(self):
    # prime at frame 1, then trigger a reset condition inside the frame%5==0 block
    for mads_enabled, cs_kwargs in [
      (True,  dict(vEgoRaw=0.5, gasPressed=True, standstill=True)),  # gas pressed
      (False, dict(vEgoRaw=0.5, standstill=True)),                   # mads disabled
      (True,  dict(vEgoRaw=1.0, standstill=False)),                  # speed above threshold
    ]:
      with self.subTest(mads_enabled=mads_enabled, cs_kwargs=cs_kwargs):
        ctrl = make_ctrl()
        self._prime(ctrl)
        ctrl.frame = 5
        run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=mads_enabled), CS=make_CS(brakePressed=False, **cs_kwargs))
        self.assertFalse(ctrl._brake_hold_primed)

  def test_none_guard_no_send(self):
    ctrl = make_ctrl()
    ctrl.frame = 0
    CS = make_CS()
    CS.es_brake_msg = None
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True), CS=CS)
    self.assertFalse(mock_bh.called)

  def test_brake_hold_flag_required(self):
    ctrl = make_ctrl(brake_hold=False)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True), CS=make_CS(standstill=True))
    self.assertFalse(mock_bh.called)

  def test_hold_frame_mod5_gating(self):
    # ES_Brake hold recomputes/sends only on frame%5 == 0
    for frame, called in [(3, False), (0, True), (5, True)]:
      with self.subTest(frame=frame):
        ctrl = make_ctrl()
        ctrl.frame = frame
        ctrl._brake_hold_primed = True
        mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                             CS=make_CS(standstill=True, brakePressed=False))
        self.assertIs(mock_bh.called, called)

  def test_mask_sends_without_es_brake_at_frame2(self):
    # Brake_Status mask (frame%2==0) and ES_Brake hold (frame%5==0) run on independent cadences;
    # _brake_hold_active carries over from a prior frame%5==0 hold
    ctrl = make_ctrl()
    ctrl._brake_hold_primed = True
    ctrl._brake_hold_active = True
    ctrl.frame = 2
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    self.assertFalse(mock_bh.called)
    self.assertTrue(mock_bsh.called)

  def test_real_packer_emits_hold_pressure(self):
    # real create_es_brake_hold + CANPacker — catches controller↔packer DBC drift the mocks can't
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    with _STEERING_PATCH, _DASHSTATUS_PATCH, _LKAS_STATE_PATCH, \
         patch(_BRAKE_STATUS_HOLD_PATCH, return_value=_DUMMY_BS_MSG):
      _, can_sends = ctrl.update(
        make_CC(), make_CC_SP(mads_enabled=True),
        make_CS(standstill=True, brakePressed=False, gasPressed=False), 0,
      )
    es_brake = [m for m in can_sends if isinstance(m[0], int) and m[0] == 0x220]
    self.assertEqual(len(es_brake), 1, "exactly one real ES_Brake frame expected")
    parser = CANParser(_DBC_NAMES[Bus.pt], [("ES_Brake", 0)], 0)
    parser.update([0, es_brake])
    self.assertEqual(parser.vl["ES_Brake"]["Brake_Pressure"], CarControllerParams.BRAKE_HOLD_PRESSURE)


# Regression for route dde08cad3a74cd94/0000004d--8360486eb1 seg 3 (2026-05-11): the carcontroller
# unconditionally injected ES_Brake=0 and the Brake_Status mask while Eyesight ACC was braking from
# rolling speed, hiding the braking module's feedback and tripping Eyesight's ~566ms Cruise_Fault
# watchdog. Invariant: when NOT holding AND NOT echoing AEB, transmit neither 0x220 nor 0x13C.
class TestACCInterferenceRegression(unittest.TestCase):

  def test_brake_hold_active_false_on_init(self):
    ctrl = make_ctrl()
    self.assertTrue(hasattr(ctrl, "_brake_hold_active"))
    self.assertFalse(ctrl._brake_hold_active)

  def test_no_es_brake_hold_when_acc_braking_not_holding(self):
    # Eyesight ACC braking from rolling speed (Brake_Pressure>0), not holding → stay out of the way
    ctrl = make_ctrl(long_control=False)
    ctrl.frame = 0
    ctrl._brake_hold_primed = False
    acc_braking_msg = {
      "AEB_Status": 0, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 40,
      "Cruise_Brake_Lights": 1, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 1,
      "Cruise_Activated": 1, "Signal3": 0,
    }
    CC = make_CC(enabled=True, long_active=False)
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC=CC, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(vEgoRaw=2.32, standstill=False, brakePressed=False,
                 gasPressed=False, es_brake_msg=acc_braking_msg),
    )
    self.assertFalse(mock_bh.called)
    self.assertFalse(mock_bsh.called)

  def test_no_brake_status_mask_when_not_holding(self):
    ctrl = make_ctrl()
    ctrl.frame = 0  # frame%2 == 0 — would otherwise trigger the mask send
    ctrl._brake_hold_primed = False
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=False, brakePressed=False, gasPressed=False, vEgoRaw=5.0),
    )
    self.assertFalse(mock_bh.called)
    self.assertFalse(mock_bsh.called)

  def test_holding_path_unchanged_regression(self):
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    self.assertTrue(mock_bh.called)
    self.assertFalse(mock_bh.call_args[0][3])  # stock Eyesight ACC — forward fault verbatim
    self.assertEqual(mock_bh.call_args[0][4], CarControllerParams.BRAKE_HOLD_PRESSURE)
    self.assertTrue(mock_bsh.called)

  def test_aeb_passthrough_keeps_both_messages(self):
    # AEB is an active intercept: echo ES_Brake (full AEB authority) AND keep the mask
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = False
    aeb_msg = {
      "AEB_Status": 4, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 600,
      "Cruise_Brake_Lights": 1, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 1,
      "Cruise_Activated": 0, "Signal3": 0,
    }
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=False, vEgoRaw=8.0, brakePressed=False,
                 gasPressed=False, es_brake_msg=aeb_msg),
    )
    self.assertTrue(mock_bh.called)
    self.assertEqual(mock_bh.call_args[0][4], 600)
    self.assertTrue(mock_bsh.called)

  def test_brake_hold_active_tracks_state(self):
    # _brake_hold_active must be True if and only if (holding or AEB)
    ctrl = make_ctrl()
    ctrl.frame = 0

    ctrl._brake_hold_primed = False
    acc_msg = {"AEB_Status": 0, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 40,
               "Cruise_Brake_Lights": 1, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 1,
               "Cruise_Activated": 1, "Signal3": 0}
    run_update_capture_both(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                            CS=make_CS(standstill=False, vEgoRaw=2.3, es_brake_msg=acc_msg))
    self.assertFalse(ctrl._brake_hold_active)

    ctrl._brake_hold_primed = True
    ctrl.frame = 0
    run_update_capture_both(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                            CS=make_CS(standstill=True, brakePressed=False, gasPressed=False))
    self.assertTrue(ctrl._brake_hold_active)

    ctrl._brake_hold_primed = False
    ctrl.frame = 0
    aeb_msg = {"AEB_Status": 4, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 600,
               "Cruise_Brake_Lights": 1, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 1,
               "Cruise_Activated": 0, "Signal3": 0}
    run_update_capture_both(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                            CS=make_CS(standstill=False, vEgoRaw=8.0, es_brake_msg=aeb_msg))
    self.assertTrue(ctrl._brake_hold_active)

  def test_brake_status_msg_none_guard(self):
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CS = make_CS(standstill=True, brakePressed=False, gasPressed=False)
    CS.brake_status_msg = None
    mock_bh, mock_bsh = run_update_capture_both(ctrl, CC_SP=make_CC_SP(mads_enabled=True), CS=CS)
    self.assertTrue(mock_bh.called)  # ES_Brake hold still goes out
    self.assertFalse(mock_bsh.called)  # mask gated on brake_status_msg presence


# AVH owns ES_Brake when openpilotLongitudinalControl=True AND longActive=False; it must yield
# (create_es_brake instead) when longActive=True. The Brake_Status mask follows the same gating.
_ES_BRAKE_PATCH = "opendbc.car.subaru.subarucan.create_es_brake"
_ES_STATUS_PATCH = "opendbc.car.subaru.subarucan.create_es_status"
_ES_DISTANCE_PATCH = "opendbc.car.subaru.subarucan.create_es_distance"


def _run_long_branch(ctrl, CC, CC_SP=None, CS=None):
  CC_SP = CC_SP or make_CC_SP()
  CS = CS or make_CS()
  with _STEERING_PATCH, _DASHSTATUS_PATCH, _LKAS_STATE_PATCH, \
       patch(_ES_STATUS_PATCH, return_value=_DUMMY_MSG), \
       patch(_ES_DISTANCE_PATCH, return_value=_DUMMY_MSG), \
       patch(_ES_BRAKE_PATCH, return_value=_DUMMY_MSG) as mock_eb, \
       patch(_BRAKE_HOLD_PATCH, return_value=_DUMMY_MSG) as mock_bh, \
       patch(_BRAKE_STATUS_HOLD_PATCH, return_value=_DUMMY_BS_MSG) as mock_bsh:
    ctrl.update(CC, CC_SP, CS, 0)
  return mock_eb, mock_bh, mock_bsh


class TestAlphaLongCoexistence(unittest.TestCase):

  def test_avh_fires_when_alpha_long_enabled_but_not_active(self):
    ctrl = make_ctrl(long_control=True, brake_hold=True)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CC = make_CC(enabled=False, long_active=False)
    mock_eb, mock_bh, _ = _run_long_branch(
      ctrl, CC, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    self.assertTrue(mock_bh.called, "create_es_brake_hold must run when alpha long enabled but inactive")
    self.assertFalse(mock_eb.called, "create_es_brake must NOT run when AVH is asserting ES_Brake")
    self.assertTrue(mock_bh.call_args[0][3])  # alpha long enabled — clear the latched fault
    self.assertEqual(mock_bh.call_args[0][4], CarControllerParams.BRAKE_HOLD_PRESSURE)

  def test_avh_yields_when_alpha_long_active(self):
    ctrl = make_ctrl(long_control=True, brake_hold=True)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CC = make_CC(enabled=True, long_active=True)
    mock_eb, mock_bh, _ = _run_long_branch(
      ctrl, CC, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    self.assertFalse(mock_bh.called, "create_es_brake_hold must yield to op long when long_active=True")
    self.assertTrue(mock_eb.called, "create_es_brake must run when long is actively in control")

  def test_brake_status_mask_under_alpha_long_when_avh_active(self):
    ctrl = make_ctrl(long_control=True, brake_hold=True)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CC = make_CC(enabled=False, long_active=False)
    _, _, mock_bsh = _run_long_branch(
      ctrl, CC, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    self.assertTrue(mock_bsh.called, "Brake_Status mask must run when AVH active under alpha long")

  def test_brake_status_mask_yields_under_alpha_long_active(self):
    ctrl = make_ctrl(long_control=True, brake_hold=True)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CC = make_CC(enabled=True, long_active=True)
    _, _, mock_bsh = _run_long_branch(
      ctrl, CC, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    self.assertFalse(mock_bsh.called, "Brake_Status mask must not run when long is active")


# AVH must never prime or hold while Eyesight ACC is engaged (cruiseState.enabled); the guard
# gates only on .enabled, not .available, and AEB echo stays independent of ACC.
class TestACCDeference(unittest.TestCase):

  def test_no_prime_when_acc_enabled(self):
    ctrl = make_ctrl()
    ctrl.frame = 1
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=0.5, brakePressed=True, cruise_enabled=True))
    self.assertFalse(ctrl._brake_hold_primed)

  def test_no_prime_when_acc_enabled_rolling(self):
    ctrl = make_ctrl()
    ctrl.frame = 1
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=1.2, brakePressed=True, cruise_enabled=True))
    self.assertFalse(ctrl._brake_hold_primed)

  def test_no_hold_when_acc_enabled(self):
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, gasPressed=False, cruise_enabled=True))
    self.assertFalse(mock_bh.called)

  def test_release_when_acc_enabled(self):
    # primed while ACC off, then ACC engages → latch clears on the frame%5==0 reset
    ctrl = make_ctrl()
    ctrl.frame = 1
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=0.5, brakePressed=True, cruise_enabled=False))
    self.assertTrue(ctrl._brake_hold_primed, "precondition: must be primed")

    ctrl.frame = 5
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(standstill=True, gasPressed=False, cruise_enabled=True))
    self.assertFalse(ctrl._brake_hold_primed)

  def test_no_brake_status_mask_when_acc_enabled(self):
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    ctrl._brake_hold_active = True
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, gasPressed=False, cruise_enabled=True),
    )
    self.assertFalse(mock_bsh.called)

  def test_mask_clears_when_acc_engages_midcadence(self):
    # frame=2: mask cadence (frame%2==0) but no state recompute (frame%5!=0); carried-over
    # active flag must not trigger the mask once ACC is engaged
    ctrl = make_ctrl()
    ctrl._brake_hold_primed = True
    ctrl._brake_hold_active = True
    ctrl.frame = 2
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, gasPressed=False, aeb_status=0, cruise_enabled=True),
    )
    self.assertFalse(mock_bsh.called)

  def test_acc_available_but_not_enabled_still_holds(self):
    # guard gates only on .enabled — main-on (available) but not engaged must still hold
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, gasPressed=False,
                                    cruise_available=True, cruise_enabled=False))
    self.assertTrue(mock_bh.called)
    self.assertEqual(mock_bh.call_args[0][4], CarControllerParams.BRAKE_HOLD_PRESSURE)

  def test_aeb_echo_unaffected_by_acc(self):
    # AEB authority is independent of ACC engagement
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = False
    aeb_msg = {
      "AEB_Status": 4, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 600,
      "Cruise_Brake_Lights": 1, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 1,
      "Cruise_Activated": 0, "Signal3": 0,
    }
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=False, vEgoRaw=8.0, brakePressed=False,
                                    gasPressed=False, es_brake_msg=aeb_msg, cruise_enabled=True))
    self.assertTrue(mock_bh.called)
    self.assertEqual(mock_bh.call_args[0][4], 600)

  def test_no_hold_when_acc_enabled_alpha_long(self):
    ctrl = make_ctrl(long_control=True)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CC = make_CC(enabled=False, long_active=False)
    mock_eb, mock_bh, _ = _run_long_branch(
      ctrl, CC, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False, cruise_enabled=True),
    )
    self.assertFalse(mock_bh.called)

  def test_real_packer_no_es_brake_when_acc_enabled(self):
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    with _STEERING_PATCH, _DASHSTATUS_PATCH, _LKAS_STATE_PATCH, \
         patch(_BRAKE_STATUS_HOLD_PATCH, return_value=_DUMMY_BS_MSG):
      _, can_sends = ctrl.update(
        make_CC(), make_CC_SP(mads_enabled=True),
        make_CS(standstill=True, brakePressed=False, gasPressed=False, cruise_enabled=True), 0,
      )
    es_brake_frames = [m for m in can_sends if isinstance(m[0], int) and m[0] == 0x220]
    self.assertEqual(len(es_brake_frames), 0, "no ES_Brake frame must be emitted when ACC is engaged")


if __name__ == "__main__":
  unittest.main()
