"""
The MIT License

Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Last updated: July 29, 2024
"""

from collections import namedtuple

from opendbc.car import DT_CTRL, structs
from opendbc.car.hyundai.values import CAR

from opendbc.sunnypilot import SunnypilotParamFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.mads_base import MadsCarStateBase

ButtonType = structs.CarState.ButtonEvent.Type

MadsDataSP = namedtuple("MadsDataSP",
                        ["enable_mads", "lat_active", "disengaging", "paused"])


class MadsCarController:
  def __init__(self):
    super().__init__()
    self.mads = MadsDataSP(False, False, False, False)

    self.lat_disengage_blink = 0
    self.lat_disengage_init = False
    self.prev_lat_active = False

    self.lkas_icon = 0
    self.lfa_icon = 0

  # display LFA "white_wheel" and LKAS "White car + lanes" when not CC.latActive
  def mads_status_update(self, CC: structs.CarControl, frame: int) -> MadsDataSP:
    enable_mads = CC.sunnypilotParams & SunnypilotParamFlags.ENABLE_MADS

    if CC.latActive:
      self.lat_disengage_init = False
    elif self.prev_lat_active:
      self.lat_disengage_init = True

    if not self.lat_disengage_init:
      self.lat_disengage_blink = frame

    paused = CC.madsEnabled and not CC.latActive
    disengaging = (frame - self.lat_disengage_blink) * DT_CTRL < 1.0 if self.lat_disengage_init else False

    self.prev_lat_active = CC.latActive

    return MadsDataSP(enable_mads, CC.latActive, disengaging, paused)

  def create_lkas_icon(self, CP: structs.CarParams, enabled: bool) -> int:
    if self.mads.enable_mads:
      lkas_icon = 2 if self.mads.lat_active else 3 if self.mads.disengaging else 1 if self.mads.paused else 0
    else:
      lkas_icon = 2 if enabled else 1

    # Override common signals for KIA_OPTIMA_G4 and KIA_OPTIMA_G4_FL
    if CP.carFingerprint in (CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL):
      lkas_icon = 3 if (self.mads.lat_active if self.mads.enable_mads else enabled) else 1

    return lkas_icon

  def create_lfa_icon(self, enabled: bool) -> int:
    if self.mads.enable_mads:
      lfa_icon = 2 if self.mads.lat_active else 3 if self.mads.disengaging else 1 if self.mads.paused else 0
    else:
      lfa_icon = 2 if enabled else 0

    return lfa_icon

  def update(self, CP: structs.CarParams, CC: structs.CarControl, frame: int):
    self.mads = self.mads_status_update(CC, frame)
    self.lkas_icon = self.create_lkas_icon(CP, CC.enabled)
    self.lfa_icon = self.create_lfa_icon(CC.enabled)


class MadsCarState(MadsCarStateBase):
  def __init__(self):
    super().__init__()
    self.main_cruise_enabled: bool = False

  def get_main_cruise(self, ret: structs.CarState, CP: structs.CarParams) -> bool:
    if any(be.type == ButtonType.mainCruise and be.pressed for be in ret.buttonEvents) and \
          (CP.sunnypilotFlags & HyundaiFlagsSP.LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE):
      self.main_cruise_enabled = not self.main_cruise_enabled

    return self.main_cruise_enabled if ret.cruiseState.available else False
