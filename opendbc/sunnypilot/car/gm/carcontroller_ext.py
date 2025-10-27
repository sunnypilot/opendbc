"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.gm import gmcan
from opendbc.car.gm.carcontroller import CarController
from opendbc.car.gm.values import CanBus, CruiseButtons
from opendbc.car.interfaces import CarStateBase
from opendbc.sunnypilot.car.gm.values_ext import GMFlagsSP
from opendbc.sunnypilot.car import crc8_pedal


def create_gas_interceptor_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  dat = packer.make_can_msg("GAS_COMMAND", 0, values)[1]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", 0, values)


class CarControllerExt(CarController):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.last_button_frame = 0

  @staticmethod
  def calc_pedal_command(accel: float, long_active: bool, v_ego: float) -> float:
    if not long_active:
      return 0.

    if accel < -0.5:
      pedal_gas = 0
    else:
      pedaloffset = np.interp(v_ego, [0., 3, 6, 30], [0.10, 0.175, 0.240, 0.240])
      pedal_gas = np.clip((pedaloffset + accel * 0.6), 0.0, 1.0)

    return pedal_gas

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators, can_sends = super().update(CC, CC_SP, CS, now_nanos)

    # Pedal interceptor logic for NON_ACC vehicles (no stock ACC / camera long)
    if self.CP.flags & GMFlagsSP.NON_ACC.value:
      # Gas/regen, brakes, and UI commands - run at 25Hz (every 4 frames)
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

      # Pedal interceptor: always send CANCEL when cruise is on
      # This keeps stock cruise from fighting us, by pretending the driver is holding CANCEL
      if CS.out.cruiseState.enabled:
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

    return actuators, can_sends