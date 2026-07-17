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


def elesys_gas_multiplier(v_ego: float) -> float:
  # FORK: pedal->accel gain falls off with speed (measured ~4.8 @ 10 m/s, ~3.5 @ 14, ~2.2 @ 18,
  # route ac35d9891f). The previous curve ([0,10,15,20] -> [0.5,1.0,1.4,2.1]) fixed the ends but
  # left the mid band on the old <=1.0 ramp: the Jul-14/15 drives (e64ef42e91/4a64f0ffb2/
  # 2418f2eb2b) delivered only ~50-65% of commanded accel at 3-14 m/s (achieved-commanded bias
  # -0.29..-0.60 m/s^2), leaving the slow ki (0.8-1.2) to grind out the rest -> the "slow off a
  # stop" feel even though pure launches tracked aTarget. This raises the middle; the PI trims
  # the remainder. Deliberately conservative vs the measured 1.5-1.7x shortfall -- revisit
  # against the next set of logs.
  return float(np.interp(v_ego, [0., 3., 6., 10., 15., 20.], [0.55, 0.85, 1.10, 1.25, 1.55, 2.20]))


class GasInterceptorCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.gas = 0.
    self.interceptor_gas_cmd = 0.

  def update(self, CC: structs.CarControl, CS: structs.CarState, gas: float, brake: float, wind_brake: float,
             packer, frame: int) -> list[CanData]:
    can_sends = []

    if self.CP_SP.enableGasInterceptor:
      # way too aggressive at low speed without this
      gas_mult = np.interp(CS.out.vEgo, [0., 10.], [0.4, 1.0])
      if self.CP.carFingerprint in HONDA_ELESYS:
        gas_mult = elesys_gas_multiplier(CS.out.vEgo)
      # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
      # This prevents unexpected pedal range rescaling
      # Sending non-zero gas when OP is not enabled will cause the PCM not to respond to throttle as expected
      # when you do enable.
      if CC.longActive:
        self.gas = float(np.clip(gas_mult * (gas - brake + wind_brake * 3 / 4), 0., 1.))
      else:
        self.gas = 0.0
      can_sends.append(create_gas_interceptor_command(packer, self.gas, frame // 2))

    return can_sends
