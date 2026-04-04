"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.honda.values import CAR
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
      # TODO-SP: Test more Nidec cars and add them to the list
      if self.CP.carFingerprint == CAR.HONDA_CLARITY:
        # mike8643 Clarity Long Tune Interpolation applied to all tested vehicles
        gas_mult = 1.0
      else:
        # gas multiplier to make pedal less touchy at low speed. The interceptor is
        # way too aggressive at low speed without this on certain models.
        gas_mult = np.interp(CS.out.vEgo, [0., 10.], [0.4, 1.0])
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
