"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False, _set=False, increase=False, decrease=False):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalt",      # ACC button, on/off
    "GRA_Typ_Hauptschalt",  # ACC button, momentary vs latching
    "GRA_Kodierinfo",       # ACC button, configuration
    "GRA_Sender",           # ACC button, CAN message originator
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen": cancel,
    "GRA_Recall": resume,
    "GRA_Neu_Setzen": _set,
    "GRA_Down_kurz": decrease,
    "GRA_Up_kurz": increase,
  })

  return packer.make_can_msg("GRA_Neu", bus, values)