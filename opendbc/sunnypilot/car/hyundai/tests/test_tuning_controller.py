"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pytest
from types import SimpleNamespace as _NS

from opendbc.sunnypilot.car.hyundai.longitudinal import config
from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalController, LongitudinalState, SPEED_BP
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.car import DT_CTRL, structs
from opendbc.car.hyundai.values import HyundaiFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState


# Build parametrization over all tuning configs once
_ALL_CONFIGS = list(config.TUNING_CONFIGS.items()) + list(config.CAR_SPECIFIC_CONFIGS.items())


def _make_controller(name: str, cfg):
  CP = _NS(carFingerprint=name, flags=0, radarUnavailable=False)
  CP_SP = _NS(flags=0)
  controller = LongitudinalController(CP, CP_SP)
  controller.car_config = cfg
  return controller, CP, CP_SP


@pytest.mark.parametrize("name,cfg", _ALL_CONFIGS)
def test_config_array_length(name, cfg):
  bp_len = len(cfg.lookahead_jerk_bp)
  assert bp_len == len(cfg.lookahead_jerk_upper_v), f"{name}: lookahead_jerk_bp and lookahead_jerk_upper_v length mismatch"
  assert bp_len == len(cfg.lookahead_jerk_lower_v), f"{name}: lookahead_jerk_bp and lookahead_jerk_lower_v length mismatch"
  assert len(cfg.upper_jerk_v) == len(SPEED_BP), f"{name}: upper_jerk_v and SPEED_BP length mismatch"
  assert len(cfg.lower_jerk_v) == len(SPEED_BP), f"{name}: lower_jerk_v and SPEED_BP length mismatch"


@pytest.mark.parametrize("name,cfg", _ALL_CONFIGS)
def test_config_array_division(name, cfg):
  controller, CP, CP_SP = _make_controller(name, cfg)
  jerk_arrays = ['upper_jerk_v', 'lower_jerk_v', 'lookahead_jerk_upper_v', 'lookahead_jerk_lower_v']

  CS = _NS(out=_NS(vEgo=5.0, aEgo=1.0), aBasis=1.0)
  CC = _NS(actuators=_NS(accel=1.0, longControlState=LongCtrlState.pid), longActive=True, hudControl=_NS(visualAlert=None))

  # Check for zeros in jerk arrays
  for array_name in jerk_arrays:
    arr = getattr(cfg, array_name, None)
    if arr is not None:
      for i, val in enumerate(arr):
        assert val != 0.0, f"{name}: {array_name}[{i}] cannot be zero"
  assert cfg.jerk_limits != 0.0, f"{name}: jerk_limits cannot be zero"

  # Check that lookahead_jerk_bp values are valid
  bp = getattr(cfg, 'lookahead_jerk_bp', None)
  if bp is not None and len(bp) > 1:
    for i in range(1, len(bp)):
      assert bp[i] > bp[i-1], f"{name}: lookahead_jerk_bp not valid at index {i}: {bp}"

  # Run config through controller and calculate
  controller.accel_cmd = 1.0
  controller.accel_last = 0.5
  controller.calculate_jerk(CC, CS, LongCtrlState.pid)
  controller._calculate_dynamic_lower_jerk(-1.0, 5.0)
  controller._calculate_lookahead_jerk(0.5, 5.0)
  controller._calculate_speed_based_jerk_limits(5.0, LongCtrlState.pid)


@pytest.mark.parametrize("name,cfg", _ALL_CONFIGS)
def test_init(name, cfg):
  controller, CP, CP_SP = _make_controller(name, cfg)
  assert isinstance(controller.tuning, LongitudinalState)
  assert controller.desired_accel == 0.0
  assert controller.actual_accel == 0.0
  assert controller.jerk_upper == 0.0
  assert controller.jerk_lower == 0.0
  assert controller.comfort_band_upper == 0.0
  assert controller.comfort_band_lower == 0.0


@pytest.mark.parametrize("name,cfg", _ALL_CONFIGS)
def test_accel_min_max_config(name, cfg):
  _controller, _CP, _CP_SP = _make_controller(name, cfg)
  assert cfg.accel_max <= 2.0, f"{name}: accel max must not exceed 2.0 m/s^2"
  assert cfg.accel_max > 0.0, f"{name}: accel max must be positive"


@pytest.mark.parametrize("name,cfg", _ALL_CONFIGS)
def test_make_jerk_flag_off(name, cfg):
  controller, CP, CP_SP = _make_controller(name, cfg)
  CC = _NS(actuators=_NS(), longActive=True)
  CS = _NS(out=_NS(vEgo=5.0, aEgo=-2.0), aBasis=0.0)
  cases = [
   # (flags, enabled, state, expected_upper, expected_lower)
    (0, None, LongCtrlState.pid,      3.0, 5.0),
    (0, None, LongCtrlState.stopping, 1.0, 5.0),
    (HyundaiFlags.CANFD, True,  LongCtrlState.pid, 3.0, 5.0),
    (HyundaiFlags.CANFD, False, LongCtrlState.pid, 3.0, 1.0),
  ]
  for flags, enabled, state, upper, lower in cases:
    controller.CP.flags = flags
    if enabled is not None:
      CC.enabled = enabled
    controller.calculate_jerk(CC, CS, state)
    print(f"[FlagOff][{name}] flags={flags}, enabled={enabled}, state={state}, " +
          f"jerk_upper={controller.jerk_upper:.3f}, jerk_lower={controller.jerk_lower:.3f}")
    assert controller.jerk_upper == upper
    assert controller.jerk_lower == lower


@pytest.mark.parametrize("name,cfg", _ALL_CONFIGS)
def test_make_jerk_flag_on(name, cfg):
  controller, CP, CP_SP = _make_controller(name, cfg)
  for mode, flag in zip([1, 2], [HyundaiFlagsSP.LONG_TUNING_DYNAMIC, HyundaiFlagsSP.LONG_TUNING_PREDICTIVE], strict=True):
    controller.long_tuning_param = mode
    controller.CP_SP.flags = flag
    controller.CP.flags = HyundaiFlags.CANFD
    CC = _NS(actuators=_NS(accel=1.0), longActive=True)
    controller.stopping = False
    CS = _NS(out=_NS(aEgo=0.8, vEgo=3.0), aBasis=0.8)
    controller.calculate_jerk(CC, CS, LongCtrlState.pid)
    print(f"[FlagOn][{name}][mode={mode}] jerk_upper={controller.jerk_upper:.3f}, jerk_lower={controller.jerk_lower:.3f}")
    assert controller.jerk_upper > 0.0
    assert controller.jerk_lower > 0.0


@pytest.mark.parametrize("name,cfg", _ALL_CONFIGS)
def test_a_value_jerk_scaling(name, cfg):
  controller, CP, CP_SP = _make_controller(name, cfg)
  controller.long_tuning_param = 1
  controller.CP_SP.flags = HyundaiFlagsSP.LONG_TUNING_DYNAMIC
  controller.CP.radarUnavailable = False
  CC = _NS(actuators=_NS(accel=1.0), longActive=True)
  print(f"[a_value][{name}][mode=1] starting accel_last:", controller.tuning.accel_last)
  # first pass: limit to jerk_upper * DT_CTRL * 2 = 0.1
  controller.jerk_upper = 0.1 / (DT_CTRL * 2)
  controller.accel_cmd = 1.0
  controller.calculate_accel(CC)
  print(f"[a_value][{name}][mode=1] pass1 actual_accel={controller.actual_accel:.5f}")
  assert abs(controller.actual_accel - 0.1) <= 1e-5

  # second pass: limit increment by new jerk_upper
  CC.actuators.accel = 0.7
  controller.jerk_upper = 0.2 / (DT_CTRL * 2)
  controller.accel_cmd = 0.7
  controller.calculate_accel(CC)
  print(f"[a_value][{name}][mode=1] pass2 actual_accel={controller.actual_accel:.5f}")
  assert abs(controller.actual_accel - 0.3) <= 1e-5


@pytest.mark.parametrize("name,cfg", _ALL_CONFIGS)
def test_make_jerk_realistic_profile(name, cfg):
  controller, CP, CP_SP = _make_controller(name, cfg)
  np.random.seed(42)
  num_points = 30
  segments = [
    np.random.uniform(0.3, 0.8, num_points//4),
    np.random.uniform(0.8, 1.6, num_points//4),
    np.random.uniform(-0.2, 0.2, num_points//4),
    np.random.uniform(-1.2, -0.5, num_points//8),
    np.random.uniform(-2.2, -1.2, num_points//8)
  ]
  accels = np.concatenate(segments)[:num_points]
  vels = np.zeros_like(accels)
  vels[0] = 5.0
  for i in range(1, len(accels)):
    vels[i] = max(0.0, min(30.0, vels[i-1] + accels[i-1] * (DT_CTRL*2)))
  CC = _NS(actuators=_NS(), longActive=True)
  CS = _NS(out=_NS())
  controller.stopping = False

  for mode, flag in zip([1, 2], [HyundaiFlagsSP.LONG_TUNING_DYNAMIC, HyundaiFlagsSP.LONG_TUNING_PREDICTIVE], strict=True):
    controller.CP_SP.flags = flag
    controller.long_tuning_param = mode
    for v, a in zip(vels, accels, strict=True):
      CS.out.vEgo = float(v)
      CS.out.aEgo = float(a)
      CS.aBasis = float(a)
      CC.actuators.accel = float(a)
      controller.calculate_jerk(CC, CS, LongCtrlState.pid)
      print(f"[realistic][mode={mode}][{name}] v={v:.2f}, a={a:.2f}, jerk_upper={controller.jerk_upper:.2f}, jerk_lower={controller.jerk_lower:.2f}")
      assert controller.jerk_upper > 0.0


@pytest.mark.parametrize("name,cfg", _ALL_CONFIGS)
def test_emergency_control_negative_accel_limit(name, cfg):
  controller, CP, CP_SP = _make_controller(name, cfg)
  CC = _NS(longActive=True)
  controller.accel_cmd = -5.0
  controller.accel_last = 0.0
  controller.emergency_control(CC)
  assert controller.actual_accel >= -3.5
