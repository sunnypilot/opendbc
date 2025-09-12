"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.sunnypilot.car import get_param


class IntelligentCruiseButtonManagementInterfaceBase:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.CC_SP = None
    self.ICBC = None
    self.frame = 0
    self.last_button_frame = 0

  @property
  def is_metric(self):
    return int(get_param(self.CP_SP.param, "IsMetric")) == 1
