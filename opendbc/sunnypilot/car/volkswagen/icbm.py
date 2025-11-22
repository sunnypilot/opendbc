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
      self.vw_can = pqcan
    else:
      self.vw_can = mqbcan

    # Coarse handling only for certain fingerprints
    self.useCoarseHandling = CP.carFingerprint in COARSE_FINGERPRINTS

    self.params = Params()
    self.is_metric = self.params.get_bool("IsMetric")
    self.coarseStep = COARSE_METRICAL if self.is_metric else COARSE_IMPERIAL
    self.speedConv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

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
        # Use coarse step if:
        # - The target speed is a multiple of 10km/h or 5mp/h
        # - The absolute difference from the set speed and the ICBM target >= 10km/h or 5mp/h
        vCruiseCluster = round(CS.out.cruiseState.speedCluster * self.speedConv)
        coarse = (self.ICBM.vTarget % self.coarseStep == 0) or (abs(vCruiseCluster - self.ICBM.vTarget) >= self.coarseStep)
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

      can_sends.append(self.vw_can.create_acc_buttons_control(**accArgs))

      self.last_button_frame = self.frame

    return can_sends
