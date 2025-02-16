from openpilot.common.numpy_fast import clip, interp
from cereal import car
from typing import Any
from openpilot.common.realtime import DT_CTRL
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from opendbc.car.hyundai.values import HyundaiFlags, CarControllerParams


LongCtrlState = car.CarControl.Actuators.LongControlState

class JerkOutput:
  def __init__(self, jerk_upper_limit, jerk_lower_limit, cb_upper, cb_lower):
    self.jerk_upper_limit = jerk_upper_limit
    self.jerk_lower_limit = jerk_lower_limit
    self.cb_upper = cb_upper
    self.cb_lower = cb_lower

class HKGLongitudinalTuning:
  def __init__(self, CP) -> None:
    self.CP = CP
    self._setup_controllers()
    self._init_state()

  def _setup_controllers(self) -> None:
    self.mpc = LongitudinalMpc(mode='acc')

    self.long_control = LongControl(self.CP)
    self.DT_CTRL = DT_CTRL
    self.params = Params()
    self.hkg_tuning = self.params.get_bool('HKGtuning')
    self.has_radar = self.params.get_bool("HyundaiRadarTracks")

  def _init_state(self) -> None:
    self.last_accel = 0.0
    self.brake_ramp = 0.0
    self.accel_last = 0.0
    self.using_e2e = False
    self.accel_raw = 0.0
    self.accel_last_jerk = 0.0
    self.jerk = 0.0
    self.jerk_count = 0.0
    self.jerk_upper_limit = 0.0
    self.jerk_lower_limit = 0.0
    self.cb_upper = self.cb_lower = 0.0


  def make_jerk(self, CS, accel, actuators):
    state = getattr(actuators, "longControlState", LongCtrlState.pid)
    # Handle cancel state to prevent cruise fault
    if not CS.out.cruiseState.enabled:
      self.jerk_upper_limit = 0.0
      self.jerk_lower_limit = 0.0
      self.cb_upper = self.cb_lower = 0.0
      self.accel_last_jerk = 0.0
      return 0.0

    if state == LongCtrlState.stopping:
      self.jerk = 1.75 / 2 - CS.out.aEgo
      return self.jerk
    else:
      current_accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      self.jerk = (current_accel - self.accel_last_jerk) / self.DT_CTRL
      self.accel_last_jerk = current_accel

    jerk_max = 5.0
    v_error = abs(CS.out.vEgo - CS.out.cruiseState.speed)

    if self.CP.flags & HyundaiFlags.CANFD.value:
      if v_error < 3.0:
        jerk_reduction = interp(v_error,
                              [0.0, 1.0, 3.0],
                              [0.3, 0.5, 1.0])
        jerk_max *= jerk_reduction

      self.jerk_upper_limit = min(max(0.5, self.jerk * 2.0), jerk_max)
      self.jerk_lower_limit = min(max(1.0, -self.jerk * 4.0), jerk_max)
      self.cb_upper = self.cb_lower = 0.0
    else:
      self.jerk_upper_limit = min(max(0.5, self.jerk * 2.0), jerk_max)
      self.jerk_lower_limit = min(max(1.0, -self.jerk * 2.0), jerk_max)
      # Make comfort band smaller when close to set speed, bigger when far away
      error_factor = interp(v_error, [0.0, 0.5, 1.0, 5.0],
                                    [0.0, 0.1, 0.5, 1.0])

      accel_factor = interp(abs(accel), [0.0, 1.0], [0.2, 0.1])
      if accel >= 0:
          self.cb_upper = clip(0.8 * error_factor + accel * accel_factor, 0, 1.0)
          self.cb_lower = clip(0.6 * error_factor + accel * accel_factor, 0, 0.8)
      else:
          self.cb_upper = clip(1.0 * error_factor + accel * accel_factor, 0, 1.2)
          self.cb_lower = clip(0.8 * error_factor + accel * accel_factor, 0, 1.2)

    return self.jerk

  def calculate_limited_accel(self, accel, actuators, CS):
    """acceleration limiting."""
    # Reset acceleration limits when cruise is disabled
    if not CS.out.cruiseState.enabled or CS.out.gasPressed:
        self.accel_last = 0.0
        self.brake_ramp = 0.0
        return accel

    self.make_jerk(CS, accel, actuators)

    # Compute accel delta vs. previous
    accel_delta = accel - self.accel_last
    ramp_rate = 0.7
    brake_aggressiveness = 0.0

    # Handle braking ramp
    if accel < 0:
      brake_ratio = clip(abs(accel / CarControllerParams.ACCEL_MIN), 0.0, 1.0)
      brake_aggressiveness = brake_ratio ** 1.5
      if CS.out.vEgo < 4.47:  # 10 mph = ~4.5 m/s
        ramp_rate = interp(
          brake_aggressiveness,
          [0.0, 0.2, 0.4, 0.6, 0.8, 1.0],
          [0.4, 0.55, 0.65, 0.85, 1.2, 1.5])
      else:
        ramp_rate = interp(
          brake_aggressiveness,
          [0.0, 0.2, 0.4, 0.6, 0.8, 1.0],
          [0.2, 0.3, 0.4, 0.6, 0.9, 1.2])

      if brake_ratio > 0.8:
        ramp_rate *= 0.8
      if self.accel_last >= 0:
        self.brake_ramp = 0.0
        ramp_rate *= 0.5

    self.brake_ramp = min(1.0, self.brake_ramp + (ramp_rate * self.DT_CTRL))

    # Apply smoothing based on speed error and brake aggressiveness
    abs_accel = abs(accel)
    if accel > 0:
      smooth_factor = interp(
          abs_accel, [0.0, 1.0, 2.0], [0.88, 0.95, 1.0]
      )
    else:
      smooth_factor = interp(
          abs_accel, [0.0, 0.3, 1.0, 2.0], [0.95, 0.85, 0.75, 0.60]
      )

    v_error = abs(CS.out.vEgo - CS.out.cruiseState.speed)
    if v_error < 3.0:  # smoothen accel if nearing target speed
      smooth_factor *= interp(
          v_error, [0.0,1.5,3.0], [0.3,0.5,0.8]
      )
    if accel < 0 and brake_aggressiveness > 0.8:
      smooth_factor *= 0.8

    # Final smoothing scaled by brake ramp
    accel_delta *= (smooth_factor * self.brake_ramp)

    # Enforce jerk limits
    accel_delta = clip(
        accel_delta,
        -self.jerk_lower_limit * self.DT_CTRL,
         self.jerk_upper_limit * self.DT_CTRL
    )

    # Gradual transition from positive accel to negative
    if accel < 0 and self.accel_last > 0:
      transition_factor = interp(
          abs(accel),
          [0,0.5,1.0,1.5,2.0],
          [0.3,0.4,0.5,0.6,0.7]
      )
      accel_delta *= transition_factor

    # Update accel
    accel = self.accel_last + accel_delta
    self.accel_last = accel
    return accel


  def apply_tune(self, CP: Any) -> None:
    CP.vEgoStopping = 0.15
    CP.vEgoStarting = 0.1
    CP.stoppingDecelRate = 0.01
    CP.startAccel = 1.0 if bool(CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV)) else 1.6
    CP.startingState = True

  def get_jerk(self) -> JerkOutput:
    return JerkOutput(
      self.jerk_upper_limit,
      self.jerk_lower_limit,
      self.cb_upper,
      self.cb_lower,
    )

class HKGLongitudinalController:
  def __init__(self, CP):
    self.jerk = None
    self.CP = CP
    self.tuning = HKGLongitudinalTuning(CP) if Params().get_bool("HKGtuning") else None

  def apply_tune(self, CP):
    if self.tuning:
      self.tuning.apply_tune(CP)

  def calculate_limited_accel(self, accel, actuators, CS):
    if self.tuning:
      return self.tuning.calculate_limited_accel(accel, actuators, CS)
    return accel

  def get_jerk(self):
    if self.tuning:
      return self.tuning.get_jerk()
    return JerkOutput(0.0, 0.0, 0.0, 0.0)
