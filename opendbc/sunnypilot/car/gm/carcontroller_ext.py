"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from opendbc.car import DT_CTRL
from opendbc.car.gm import gmcan
from opendbc.car.gm.values import CanBus, CruiseButtons
from opendbc.sunnypilot.car.gm.values_ext import GMFlagsSP
from opendbc.sunnypilot.car import create_gas_interceptor_command


class GasInterceptorCarController:
  def __init__(self, CP, CP_SP):
    # last_button_frame is separate from main controller timing
    self.last_button_frame = 0
    self.CP = CP
    self.CP_SP = CP_SP
    self.frame = 0

  @staticmethod
  def calc_pedal_command(accel: float, long_active: bool, v_ego: float) -> float:
    if not long_active:
      return 0.

    if accel < -0.5:
      pedal_gas = 0.
    else:
      pedaloffset = np.interp(v_ego, [0., 3., 6., 30.], [0.10, 0.175, 0.240, 0.240])
      pedal_gas = np.clip((pedaloffset + accel * 0.6), 0.0, 1.0)

    return pedal_gas

  def extend_with_interceptor(self, CC, CS, actuators, can_sends):
    """
    Inject NON_ACC / pedal interceptor behavior.
    This mutates can_sends in-place, and returns nothing.
    """

    # Only run this path for NON_ACC (camera long / pedal cars) with a detected interceptor
    if not (self.CP.enableGasInterceptorDEPRECATED and (self.CP_SP.flags & GMFlagsSP.NON_ACC)):
      return

    # Gas/regen/longitudinal pedal command @25Hz (every 4 frames)
    if self.frame % 4 == 0:
      interceptor_gas_cmd = 0.0

      if CC.longActive:
        # openpilot is responsible for longitudinal, so generate pedal command
        interceptor_gas_cmd = self.calc_pedal_command(
          actuators.accel,
          CC.longActive,
          CS.out.vEgo,
        )

      idx = (self.frame // 4) % 4

      # Always send the pedal command to keep the interceptor alive
      can_sends.append(
        create_gas_interceptor_command(
          self.packer_pt,
          interceptor_gas_cmd,
          idx,
        )
      )

    # While cruise is enabled, continuously send CANCEL to prevent stock ACC from taking over
    if self.CP.enableGasInterceptorDEPRECATED and CS.out.cruiseState.enabled:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.04:
        self.last_button_frame = self.frame
        can_sends.append(
          gmcan.create_buttons(
            self.packer_pt,
            CanBus.POWERTRAIN,
            (CS.buttons_counter + 1) % 4,
            CruiseButtons.CANCEL,
          )
        )