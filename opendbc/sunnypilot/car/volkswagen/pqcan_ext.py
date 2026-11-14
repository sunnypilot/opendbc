"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


def create_acc_button_press(packer, bus, gra_stock_values, button_signal):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalt",
    "GRA_Typ_Hauptschalt",
    "GRA_Kodierinfo",
    "GRA_Sender",
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    button_signal: 1,
  })

  return packer.make_can_msg("GRA_Neu", bus, values)
