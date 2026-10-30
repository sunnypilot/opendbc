"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False, _set=False, increase=False, decrease=False):
  values = {s: gra_stock_values[s] for s in [
    "LS_Hauptschalter",
    "LS_Typ_Hauptschalter",
    "LS_Codierung",
    "LS_Tip_Stufe_2",
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "LS_Abbrechen": cancel,
    "LS_Tip_Wiederaufnahme": resume,
    "LS_Tip_Setzen": _set,
    "LS_Tip_Runter": decrease,
    "LS_Tip_Hoch": increase,
  })

  return packer.make_can_msg("LS_01", bus, values)
