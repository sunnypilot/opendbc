"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from collections import deque
from dataclasses import dataclass

from opendbc.car import structs, DT_CTRL
from opendbc.car.interfaces import CarStateBase
from opendbc.car.hyundai.values import CarControllerParams, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.longitudinal.config import SCC14Family
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_car_config, jerk_limited_integrator
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState

DT = DT_CTRL * 2  # every SCC12 frame

# what the radar's own SCC sends the ESC for the same request, from the Kia Niro EV's stock cruise
ESC_JERK_LOWER = 0.5
ESC_JERK_LOWER_LEAD = 1.0
ESC_LEAD_GAP_A = 20.0
ESC_LEAD_GAP_T = 0.44
ESC_CLOSE_T_UPPER = 0.9
ESC_CLOSE_T_LOWER = 0.5
ESC_TRACK_BELOW = -0.5
ESC_TRACK_LAUNCH_V = 2.0
ESC_RATE_T = 0.1
ESC_BAND_LOWER_BP = [-2.5, -1.85, -1.25, -0.8, -0.45, -0.05, 0.05, 0.2, 0.45, 0.7]
ESC_BAND_LOWER_V = [0.07, 0.155, 0.19, 0.16, 0.145, 0.145, 0.065, 0.07, 0.02, 0.0]
ESC_BAND_UPPER_BP = [0.05, 0.2, 0.45, 0.8]
ESC_BAND_UPPER_V = [0.0, 0.024, 0.063, 0.085]
ESC_CANFD_BAND_PULL_AWAY = 1.0
ISO_JERK_LOWER_BP = [5.0, 20.0]
ISO_JERK_LOWER_V = [5.0, 2.5]


@dataclass
class LongitudinalState:
  desired_accel: float = 0.0
  actual_accel: float = 0.0
  accel_last: float = 0.0
  jerk_upper: float = 0.0
  jerk_lower: float = 0.0
  comfort_band_upper: float = 0.0
  comfort_band_lower: float = 0.0
  stopping: bool = False
  send_scc14: bool = True


class LongitudinalController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP = CP
    self.CP_SP = CP_SP

    self.tuning = LongitudinalState()
    self.car_config = get_car_config(CP)

    self.accel_cmd = 0.0
    self.accel_cmd_hist: deque[float] = deque(maxlen=round(ESC_RATE_T / DT))
    self.lead_distance_bars = 2
    self.desired_accel = 0.0
    self.actual_accel = 0.0
    self.accel_last = 0.0
    self.jerk_upper = 0.0
    self.jerk_lower = 0.0
    self.comfort_band_upper = 0.0
    self.comfort_band_lower = 0.0
    self.stopping = False

  @property
  def enabled(self) -> bool:
    return bool(self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING)

  @property
  def scc14(self) -> SCC14Family:
    return self.car_config.scc14 if self.enabled else SCC14Family.RATE

  def _jerk_caps(self, velocity: float, long_control_state: LongCtrlState) -> tuple[float, float]:
    upper = float(np.interp(velocity, [0.0, 5.0, 20.0], [2.0, 3.0, 2.0])) if long_control_state == LongCtrlState.pid else 0.5
    return upper, float(np.interp(velocity, ISO_JERK_LOWER_BP, ISO_JERK_LOWER_V))

  def _lead_within(self, distance: float = float('inf')) -> bool:
    lead = getattr(self, 'lead_one', None)
    return lead is not None and bool(lead.status) and lead.dRel < distance

  def _calculate_esc_jerk(self, velocity: float, long_control_state: LongCtrlState) -> tuple[float, float]:
    following = ESC_LEAD_GAP_A + ESC_LEAD_GAP_T * self.lead_distance_bars * velocity
    free, near = self.car_config.jerk_lower_floor or (ESC_JERK_LOWER, ESC_JERK_LOWER_LEAD)
    floor_lower = near if self._lead_within(following) else free
    if self.stopping:
      return self.car_config.jerk_upper_floor, floor_lower
    hist = self.accel_cmd_hist
    rate = (self.accel_cmd - hist[0]) / (DT * len(hist)) if hist else 0.0
    # the request's own rate only while releasing braking or launching
    if self.accel_last >= ESC_TRACK_BELOW and velocity >= ESC_TRACK_LAUNCH_V:
      rate = 0.0
    gap = self.accel_cmd - self.accel_last
    upper = max(self.car_config.jerk_upper_floor, rate, gap / ESC_CLOSE_T_UPPER)
    lower = max(floor_lower, -rate, -gap / ESC_CLOSE_T_LOWER)
    upper_cap, lower_cap = self._jerk_caps(velocity, long_control_state)
    return min(upper, upper_cap), min(lower, lower_cap)

  def _calculate_stepped_jerk(self, velocity: float, long_control_state: LongCtrlState) -> tuple[float, float]:
    up_acc, lo_acc, up_brk, lo_brk = self.car_config.stepped_jerk
    upper, lower = (up_acc, lo_acc) if self.accel_last > 0.0 else (up_brk, lo_brk)
    upper_cap, lower_cap = self._jerk_caps(velocity, long_control_state)
    return min(upper, upper_cap), min(lower, lower_cap)

  def calculate_jerk(self, CC: structs.CarControl, CS: CarStateBase, long_control_state: LongCtrlState) -> None:
    if not self.enabled:
      self.jerk_upper = 3.0 if long_control_state == LongCtrlState.pid else 1.0
      self.jerk_lower = 5.0
    elif not CC.longActive:
      self.jerk_upper = self.jerk_lower = 0.0
    else:
      if self.scc14 == SCC14Family.RATE:
        self.jerk_upper, self.jerk_lower = self._calculate_esc_jerk(CS.out.vEgo, long_control_state)
      elif self.scc14 == SCC14Family.STEPPED:
        self.jerk_upper, self.jerk_lower = self._calculate_stepped_jerk(CS.out.vEgo, long_control_state)
      else:
        self.jerk_upper, self.jerk_lower = self.car_config.scc14_jerk
      if self.CP.radarUnavailable:
        self.jerk_lower = 5.0

  def calculate_accel(self, CC: structs.CarControl) -> None:
    if not self.enabled:
      self.desired_accel = self.actual_accel = self.accel_cmd
      return

    if not CC.longActive:
      self.desired_accel = self.actual_accel = self.accel_last = 0.0
      return

    self.desired_accel = 0.0 if self.stopping else float(np.clip(self.accel_cmd, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    # radars that send no jerk fields send the request itself
    if not self.CP.radarUnavailable and self.scc14 not in (SCC14Family.ZEROS, SCC14Family.NONE):
      self.actual_accel = jerk_limited_integrator(self.desired_accel, self.accel_last, self.jerk_upper, self.jerk_lower, DT)
    else:
      self.actual_accel = self.desired_accel
    self.accel_last = self.actual_accel

  def calculate_comfort_band(self, CC: structs.CarControl, CS: CarStateBase) -> None:
    self.comfort_band_upper = self.comfort_band_lower = 0.0
    if not self.enabled or not CC.longActive:
      return
    if self.CP.flags & HyundaiFlags.CANFD:
      if self._lead_within() and self.lead_one.vRel < ESC_CANFD_BAND_PULL_AWAY:
        self.comfort_band_upper, self.comfort_band_lower = self.car_config.canfd_band
    elif self.scc14 == SCC14Family.RATE and self._lead_within() and not self.stopping:
      self.comfort_band_upper = float(np.interp(self.actual_accel, ESC_BAND_UPPER_BP, ESC_BAND_UPPER_V))
      self.comfort_band_lower = float(np.interp(self.actual_accel, ESC_BAND_LOWER_BP, ESC_BAND_LOWER_V))

  def get_tuning_state(self) -> None:
    self.tuning = LongitudinalState(
      desired_accel=self.desired_accel,
      actual_accel=self.actual_accel,
      accel_last=self.accel_last,
      jerk_upper=self.jerk_upper,
      jerk_lower=self.jerk_lower,
      comfort_band_upper=self.comfort_band_upper,
      comfort_band_lower=self.comfort_band_lower,
      stopping=self.stopping,
      send_scc14=self.scc14 != SCC14Family.NONE,
    )

  def update(self, CC: structs.CarControl, CS: CarStateBase) -> None:
    long_control_state = CC.actuators.longControlState
    self.accel_cmd = CC.actuators.accel
    self.lead_distance_bars = CC.hudControl.leadDistanceBars
    self.stopping = long_control_state == LongCtrlState.stopping

    self.calculate_jerk(CC, CS, long_control_state)
    self.calculate_accel(CC)
    self.calculate_comfort_band(CC, CS)
    self.get_tuning_state()

    if CC.longActive:
      self.accel_cmd_hist.append(self.accel_cmd)
    else:
      self.accel_cmd_hist.clear()
