"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections import namedtuple

from opendbc.car import structs

MadsDataSP = namedtuple("MadsDataSP",
                        ["use_lka_mode"])


class MadsCarController:
  def __init__(self):
    super().__init__()
    self.mads = MadsDataSP(False)

  @staticmethod
  def mads_status_update(CC_SP: structs.CarControlSP) -> MadsDataSP:
    use_lka_mode = CC_SP.mads.available

    return MadsDataSP(use_lka_mode)

  def update(self, CC_SP: structs.CarControlSP) -> None:
    self.mads = self.mads_status_update(CC_SP)
