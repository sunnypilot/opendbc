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
from opendbc.sunnypilot.car.volkswagen import mlbcan_ext as mlbcan
from openpilot.common.params import Params
from openpilot.common.constants import CV
from enum import Enum, auto
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState
ButtonType = structs.CarState.ButtonEvent.Type

SEND_REPETITIONS = 2
COOLDOWN_TIME = 0.2

COARSE_METRICAL = 10 # km/h
COARSE_IMPERIAL = 5 # mp/h

COARSE_FINGERPRINTS = [
  "SEAT_ATECA_MK1",
  "VOLKSWAGEN_GOLF_MK7"
]

class VWICBMState(Enum):
  IDLE = auto()
  SPAM = auto()
  COOLDOWN = auto()

class VWICBMHelper:
  def __init__(self):
    self.state = VWICBMState.IDLE
    self.button = None
    self.baseCounter = None
    self.repetition = None

class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS_EXT = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS_EXT = mlbcan
    else:
      self.CCS_EXT = mqbcan

    # Coarse handling only for certain fingerprints
    self.useCoarseHandling = CP.carFingerprint in COARSE_FINGERPRINTS

    self.params = Params()
    self.isMetric = self.params.get_bool("IsMetric")
    self.coarseStep = COARSE_METRICAL if self.isMetric else COARSE_IMPERIAL
    self.speedConv = CV.MS_TO_KPH if self.isMetric else CV.MS_TO_MPH
    self.icbmHelper = VWICBMHelper()

  def update(self, CS, CC_SP, packer, CAN, frame, recievedGRA) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame

    def getButtonToPress():
      # Check if the next coarse cruise speed would overshoot the target. If not: use coarse.
      if self.useCoarseHandling:
        vCruiseCluster = round(CS.out.cruiseState.speedCluster * self.speedConv)
        if self.ICBM.sendButton == SendButtonState.increase:
            coarseTarget = (vCruiseCluster // self.coarseStep + 1) * self.coarseStep
            coarse = coarseTarget <= self.ICBM.vTarget
            return ButtonType.accelCruise if coarse else ButtonType.resumeCruise
        else:
            coarseTarget = (vCruiseCluster // self.coarseStep) * self.coarseStep
            if vCruiseCluster % self.coarseStep == 0:
                coarseTarget -= self.coarseStep
            coarse = coarseTarget >= self.ICBM.vTarget
            return ButtonType.decelCruise if coarse else ButtonType.setCruise
      # No coarse handling
      else:
        return ButtonType.accelCruise if self.ICBM.sendButton == SendButtonState.increase else ButtonType.decelCruise

    def getAccArgs():
      accArgs = {
        "packer": packer,
        "bus": CAN,
        "gra_stock_values": CS.gra_stock_values,
        "increase": self.icbmHelper.button == ButtonType.accelCruise,
        "decrease": self.icbmHelper.button == ButtonType.decelCruise,
        "resume"  : self.icbmHelper.button == ButtonType.resumeCruise,
        "_set"    : self.icbmHelper.button == ButtonType.setCruise
      }

      # Patch counter
      accArgs["gra_stock_values"].update({
        "COUNTER": self.icbmHelper.baseCounter + self.icbmHelper.repetition
      })

      return accArgs

    # IDLE
    if self.icbmHelper.state == VWICBMState.IDLE:

      # ICBM wants to send and we just received a GRA message
      if self.ICBM.sendButton != SendButtonState.none and recievedGRA:

        # Prepare stuff for spamming
        self.icbmHelper.button = getButtonToPress()
        self.icbmHelper.state = VWICBMState.SPAM
        self.icbmHelper.repetition = 0
        self.icbmHelper.baseCounter = CS.gra_stock_values["COUNTER"]

        # Send first frame
        self.last_button_frame = frame
        can_sends.append(self.CCS_EXT.create_acc_buttons_control(**getAccArgs()))

    # SPAM
    elif self.icbmHelper.state == VWICBMState.SPAM:

      self.icbmHelper.repetition += 1

      # GRA just sent a new frame or we reached the limit: Abort the spam
      if recievedGRA or self.icbmHelper.repetition > SEND_REPETITIONS:
        self.icbmHelper.state = VWICBMState.COOLDOWN

      # Spam once more
      else:
        self.last_button_frame = frame
        can_sends.append(self.CCS_EXT.create_acc_buttons_control(**getAccArgs()))

    # COOLDOWN
    elif self.icbmHelper.state == VWICBMState.COOLDOWN:
      self.icbmHelper.state = VWICBMState.IDLE if (self.frame - self.last_button_frame) * DT_CTRL > COOLDOWN_TIME else VWICBMState.COOLDOWN

    return can_sends
  