"""
Copyright ©️ SunnyPilot & a number of other contributors.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.volkswagen import pqcan, mqbcan
from opendbc.car.volkswagen.values import VolkswagenFlags
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else mqbcan

  def update(self, CC_SP, CS, packer, frame, bus) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame

    increase = self.ICBM.sendButton == SendButtonState.increase
    decrease = self.ICBM.sendButton == SendButtonState.decrease

    # Send speed adjustment buttons only when:
    # - Cruise is already engaged (to avoid unintended activation)
    # - Button command is requested (increase or decrease)
    # - Sufficient time has passed since last button (200ms debounce)
    if CS.out.cruiseState.enabled and (increase or decrease):
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        can_sends.append(self.CCS.create_acc_buttons_control(packer, bus, CS.gra_stock_values, speed_up=increase, speed_down=decrease))
        self.last_button_frame = self.frame

    return can_sends
