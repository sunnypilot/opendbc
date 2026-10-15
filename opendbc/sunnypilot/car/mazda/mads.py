from enum import StrEnum

from opendbc.can import CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.sunnypilot.mads_base import MadsCarStateBase

ButtonType = structs.CarState.ButtonEvent.Type

# Require fresh, disabled standby feedback before ON can become a toggle event.
MADS_ON_STANDBY_FRAMES = round(0.5 / DT_CTRL)


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.mads_available = False
    self.mads_enabled = False
    self.mads_feedback_age = MADS_ON_STANDBY_FRAMES
    self.mads_standby_frames = 0
    self.main_on_button = False
    self.main_on_lkas_button = False
    self.prev_main_on_lkas_button = False

  def update_mads_feedback(self, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    self.mads_available = CC_SP.mads.available
    self.mads_enabled = CC_SP.mads.enabled or CC_SP.mads.active or CC.latActive
    self.mads_feedback_age = 0

  def main_on_lkas_events(self, ret: structs.CarState, btns) -> list[structs.CarState.ButtonEvent]:
    self.mads_feedback_age = min(self.mads_feedback_age + 1, MADS_ON_STANDBY_FRAMES)
    on_pressed = btns["MODE_Y"] == 1
    brake_restricted = self.CP.alternativeExperience & (ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE |
                                                       ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE)
    standby = (self.mads_available and not self.mads_enabled and
               self.mads_feedback_age < MADS_ON_STANDBY_FRAMES and ret.cruiseState.available and self.out.cruiseState.available and
               not ret.cruiseState.enabled and not self.out.cruiseState.enabled and
               not (btns["CAN_OFF"] or self.accel_button or btns["SET_P"] or self.decel_button) and
               not (brake_restricted and (ret.brakePressed or ret.regenBraking)))
    self.mads_standby_frames = min(self.mads_standby_frames + 1, MADS_ON_STANDBY_FRAMES) if standby else 0

    self.prev_main_on_lkas_button = self.main_on_lkas_button
    if not on_pressed:
      self.main_on_lkas_button = False
    elif not self.main_on_button and self.mads_standby_frames >= MADS_ON_STANDBY_FRAMES:
      # The shared MADS handler toggles on a pressed LKAS event. Emit once, only
      # from settled disabled standby, then wait for feedback before another request.
      self.main_on_lkas_button = True
      self.mads_standby_frames = 0
    self.main_on_button = on_pressed
    return create_button_events(self.main_on_lkas_button, self.prev_main_on_lkas_button, {True: ButtonType.lkas})

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    ret.buttonEvents = [*ret.buttonEvents, *self.main_on_lkas_events(ret, can_parsers[Bus.pt].vl["CRZ_BTNS"])]
