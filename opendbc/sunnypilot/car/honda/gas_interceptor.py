"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.honda.values import HONDA_ELESYS
from opendbc.sunnypilot.car import create_gas_interceptor_command

# Single source of truth for the pedal gain curve. dynamic_tuning.py learns a
# multiplicative correction on this same grid, so keep the two in step.
ELESYS_GAS_BP = [0., 3., 6., 10., 15., 20.]
ELESYS_GAS_V = [0.55, 0.85, 1.20, 1.55, 1.95, 2.75]


def elesys_gas_multiplier(v_ego: float) -> float:
  # FORK: pedal->accel gain falls off with speed (measured ~4.8 @ 10 m/s, ~3.5 @ 14, ~2.2 @ 18,
  # route ac35d9891f). The previous curve ([0,10,15,20] -> [0.5,1.0,1.4,2.1]) fixed the ends but
  # left the mid band on the old <=1.0 ramp: the Jul-14/15 drives (e64ef42e91/4a64f0ffb2/
  # 2418f2eb2b) delivered only ~50-65% of commanded accel at 3-14 m/s (achieved-commanded bias
  # -0.29..-0.60 m/s^2), leaving the slow ki (0.8-1.2) to grind out the rest -> the "slow off a
  # stop" feel even though pure launches tracked aTarget. This raises the middle; the PI trims
  # the remainder.
  #
  # 2026-08 revision, against the full log set. The shortfall was still there and the top of the
  # curve was the worst part of it. Plant identification on settled frames (target steady >= 1.5 s,
  # demand = aEgo + g*sin(pitch) + aero(v) regressed on the interceptor command, ~200k frames over
  # three routes on two different tunes) says the command actually needed was this multiple of
  # what was sent:  3-6 m/s 1.38   6-10 1.30-1.47   10-15 1.59-1.70   15-20 1.51-1.52   20+ 1.50-1.75.
  # Nothing physical was capping it: actuatorsOutput.gas never once reached 1.0, and never even
  # exceeded 0.9, on any of the 13 engaged routes (mean 0.13-0.22).
  #
  # So the top three breakpoints go up ~1.25x -- 1.10/1.25/1.55/2.20 -> 1.20/1.55/1.95/2.75 -- NOT
  # the full measured ratio. One measured step, because the PI and the pitch feedforward close part
  # of the rest and a full-ratio jump is untested open-loop. Expect the next log's per-band ratio
  # near 1.15-1.25; if 15-20 m/s is still >= 1.4, take a second 1.25x step.
  #
  # This is also why PEDAL_GAIN_MAX stays at 1.8 rather than being raised. Replaying the tuner
  # with persistence chained across all 13 drives in route order, the learner PINS at 1.800 on the
  # 10 and 15 m/s breakpoints and stays pinned, while the 20 m/s cell -- where the requirement is
  # largest -- barely moves (1.099 live / 1.046 persisted), because LEARN_MIN_CMD rejects almost
  # every highway frame. Raising the clamp would just let it chase the same gap more slowly. Fixing
  # the base curve is what gets the residual back inside the clamp.
  #
  # With HondaDynamicTuningEnabled this curve is the *starting point* only: the tuner learns a
  # per-breakpoint correction on top of it and persists it across drives.
  return float(np.interp(v_ego, ELESYS_GAS_BP, ELESYS_GAS_V))


class GasInterceptorCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.gas = 0.
    self.interceptor_gas_cmd = 0.

  def update(self, CC: structs.CarControl, CS: structs.CarState, gas: float, brake: float, wind_brake: float,
             packer, frame: int, tuner=None) -> list[CanData]:
    can_sends = []

    if self.CP_SP.enableGasInterceptor:
      # way too aggressive at low speed without this
      gas_mult = np.interp(CS.out.vEgo, [0., 10.], [0.4, 1.0])
      if self.CP.carFingerprint in HONDA_ELESYS:
        gas_mult = elesys_gas_multiplier(CS.out.vEgo)

      # Learned correction and, when the PCM crossfade is enabled, the pedal's share of the
      # request. Both are 1.0 when the dynamic tuner is off, so this is a no-op by default.
      pedal_share = 1.0
      if tuner is not None:
        gas_mult *= tuner.pedal_gain_at(CS.out.vEgo)
        pedal_share = tuner.pedal_authority(CS.out.vEgo)

      # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
      # This prevents unexpected pedal range rescaling
      # Sending non-zero gas when OP is not enabled will cause the PCM not to respond to throttle as expected
      # when you do enable.
      if CC.longActive:
        self.gas = float(np.clip(gas_mult * pedal_share * (gas - brake + wind_brake * 3 / 4), 0., 1.))
      else:
        self.gas = 0.0
      can_sends.append(create_gas_interceptor_command(packer, self.gas, frame // 2))

      if tuner is not None:
        tuner.update_pedal(CC, CS, self.gas)

    return can_sends
