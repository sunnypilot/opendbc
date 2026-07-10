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
        # FORK: measured pedal->accel gain (grade+lag corrected, route ac35d9891f) is ~4.8 @ 10 m/s
        # but falls to ~3.5 @ 14 and ~2.2 @ 18 m/s. The capped 1.0 multiplier under-gassed at speed,
        # leaving the slow ki (0.5) to grind out the error -> sluggish accel / pedal-lag feel.
        # low end 0.4 -> 0.5: launches delivered ~0.5 of planned accel at 1-3 m/s (uphill starts,
        # route fb142b2417); +25% pedal at launch, PI trims the rest. Revisit after next drives.
        gas_mult = np.interp(CS.out.vEgo, [0., 10., 15., 20.], [0.5, 1.0, 1.4, 2.1])
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
