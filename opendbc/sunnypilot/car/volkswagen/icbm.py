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
from openpilot.common.params import Params
from openpilot.common.constants import CV
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

COARSE_METRICAL = 10 # km/h
COARSE_IMPERIAL = 5 # mp/h

COARSE_FINGERPRINTS = [
  "SEAT_ATECA_MK1",
  "VOLKSWAGEN_GOLF_MK7"
]

class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    else:
      self.CCS = mqbcan

    # Coarse handling only for certain fingerprints
    self.useCoarseHandling = CP.carFingerprint in COARSE_FINGERPRINTS

    self.params = Params()
    self.isMetric = self.params.get_bool("IsMetric")
    self.coarseStep = COARSE_METRICAL if self.isMetric else COARSE_IMPERIAL
    self.speedConv = CV.MS_TO_KPH if self.isMetric else CV.MS_TO_MPH

  def update(self, CS, CC_SP, packer, frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame

    if self.ICBM.sendButton != SendButtonState.none and (self.frame - self.last_button_frame) * DT_CTRL > 0.2:

      accArgs = {
        "packer": packer,
        "bus": CAN,
        "gra_stock_values": CS.gra_stock_values
      }

      if self.useCoarseHandling:
        # Check if the next coarse cruise speed would overshoot the target. If not: use coarse.
        vCruiseCluster = round(CS.out.cruiseState.speedCluster * self.speedConv)
        coarse = self.useCoarse(vCruiseCluster)
        accArgs.update({
          "increase": (self.ICBM.sendButton == SendButtonState.increase) and (coarse),
          "decrease": (self.ICBM.sendButton == SendButtonState.decrease) and (coarse),
          "resume"  : (self.ICBM.sendButton == SendButtonState.increase) and (not coarse),
          "_set"    : (self.ICBM.sendButton == SendButtonState.decrease) and (not coarse)
        })
      else:
        accArgs.update({
          "increase": self.ICBM.sendButton == SendButtonState.increase,
          "decrease": self.ICBM.sendButton == SendButtonState.decrease,
        })

      can_sends.append(self.CCS.create_acc_buttons_control(**accArgs))
      self.last_button_frame = self.frame

    return can_sends

  def useCoarse(self, currentCruise):
    if self.ICBM.sendButton == SendButtonState.increase:
        coarseTarget = (currentCruise // self.coarseStep + 1) * self.coarseStep
        return coarseTarget <= self.ICBM.vTarget
    else:
        coarseTarget = (currentCruise // self.coarseStep) * self.coarseStep
        if currentCruise % self.coarseStep == 0:
            coarseTarget -= self.coarseStep
        return coarseTarget >= self.ICBM.vTarget