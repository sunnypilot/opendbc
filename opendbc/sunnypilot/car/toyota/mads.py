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

from opendbc.car import structs

from opendbc.sunnypilot.mads_base import MadsCarStateBase

ButtonType = structs.CarState.ButtonEvent.Type


class MadsCarState(MadsCarStateBase):
  distance_button_events: list[structs.CarState.ButtonEvent]
  lkas_button_events: list[structs.CarState.ButtonEvent]

  def __init__(self):
    super().__init__()
    self.lta_status = False
    self.prev_lta_status = False
    self.lta_status_active = False

    self.distance_button_events = []
    self.lkas_button_events = []

  def get_lkas_button(self, cp_cam):
    self.lta_status = cp_cam.vl["LKAS_HUD"]["SET_ME_X02"]
    if ((self.prev_lta_status == 16 and self.lta_status == 0) or
        (self.prev_lta_status == 0 and self.lta_status == 16)) and \
       not self.lta_status_active:
      self.lta_status_active = True

    if self.lta_status_active:
      lkas_button = self.lta_status
    else:
      lkas_button = cp_cam.vl["LKAS_HUD"]["LKAS_STATUS"]

    return lkas_button

  def create_lkas_button_events(self, cur_btn: int, prev_btn: int,
                                buttons_dict: dict[int, structs.CarState.ButtonEvent.Type]) -> list[structs.CarState.ButtonEvent]:
    events: list[structs.CarState.ButtonEvent] = []

    if cur_btn == prev_btn:
      return events

    if self.lta_status_active:
      state_changes = [
        {"pressed": bool(prev_btn == 16 and cur_btn == 0)},
        {"pressed": bool(prev_btn == 0 and cur_btn == 16)},
      ]
    else:
      state_changes = [
        {"pressed": bool(not prev_btn and cur_btn)},
        {"pressed": bool(prev_btn == 1 and not cur_btn)},
      ]

    for change in state_changes:
      if change["pressed"]:
        events.append(structs.CarState.ButtonEvent(pressed=change["pressed"],
                                                   type=buttons_dict.get(cur_btn, ButtonType.unknown)))
    return events
