"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.volkswagen.values import VolkswagenFlags
from opendbc.sunnypilot.car.volkswagen import mqbcan_ext as mqbcan
from opendbc.sunnypilot.car.volkswagen import pqcan_ext as pqcan
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

COARSE_METRICAL = 10 # km/h
COARSE_IMPERIAL = 5 # mp/h

class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.vw_can = pqcan if self.CP.flags & VolkswagenFlags.PQ else mqbcan

  def update(self, CS, CC_SP, packer, frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame

    # ICBM wants to send a Button != NONE
    if self.ICBM.sendButton != SendButtonState.none and (self.frame - self.last_button_frame) * DT_CTRL > 0.2:

      # Check coarse
      coarse = abs(self.ICBM.vError) >= COARSE_METRICAL # TODO check when to use coarse

      # Create Args for VW GRA ACC Controller
      accArgs = {
        "packer": packer,
        "bus": CAN,
        "gra_stock_values": CS.gra_stock_values,
        "increase": (self.ICBM.sendButton == SendButtonState.increase) and (coarse),
        "decrease": (self.ICBM.sendButton == SendButtonState.decrease) and (coarse),
        "resume"  : (self.ICBM.sendButton == SendButtonState.increase) and (not coarse),
        "_set"    : (self.ICBM.sendButton == SendButtonState.decrease) and (not coarse)
      }

      # Create Command
      can_sends.append(self.vw_can.create_acc_buttons_control(**accArgs))

      # Update Button Frame
      self.last_button_frame = self.frame

    return can_sends
