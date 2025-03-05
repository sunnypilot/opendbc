import numpy as np
from openpilot.common.numpy_fast import clip, interp
from typing import Any
from opendbc.car import structs
from opendbc.car import DT_CTRL
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from opendbc.car.hyundai.values import CAR, HyundaiFlags, CarControllerParams


LongCtrlState = structs.CarControl.Actuators.LongControlState

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
    self._mode_setup()

  def _setup_controllers(self) -> None:
    self.mpc = LongitudinalMpc
    self.long_control = LongControl(self.CP)
    self.DT_CTRL = DT_CTRL
    self.params = Params()
    self.hkg_tuning = self.params.get_bool('HKGtuning')
    self.has_radar = self.params.get_bool("HyundaiRadarTracks")

  def _init_state(self) -> None:
    self.last_accel = 0.0
    self.brake_ramp = 0.0
    self.accel_last = 0.0
    self.accel_last_jerk = 0.0
    self.jerk = 0.0
    self.jerk_count = 0.0
    self.jerk_upper_limit = 0.0
    self.jerk_lower_limit = 0.0
    self.cb_upper = self.cb_lower = 0.0
    self.last_decel_time = 0.0
    self.min_cancel_delay = 0.1

  def _mode_setup(self) -> None:
    self.prev_mode = 'acc'  # Default mode
    self.current_mode = 'acc'
    self.mode_transition_filter = FirstOrderFilter(0.0, 0.5, DT_CTRL)  # 0.5s time constant
    self.mode_transition_timer = 0.0
    self.mode_transition_duration = 1.0  # time to transition
    self.transitioning = False

  def update_mpc_mode(self, sm):
    """Update MPC mode with transition handling."""
    new_mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    # Detect mode change
    if new_mode != self.current_mode:
      self.prev_mode = self.current_mode
      self.transitioning = True
      self.mode_transition_timer = 0.0
      self.mode_transition_filter.x = self.accel_last

      # Update the MPC mode directly
      self.mpc.mode = new_mode
      self.current_mode = new_mode

    # Update transition state
    if self.transitioning:
      self.mode_transition_timer += DT_CTRL
      if self.mode_transition_timer >= self.mode_transition_duration:
        self.transitioning = False

  def make_jerk(self, CS, actuators):
    state = getattr(actuators, "longControlState", LongCtrlState.pid)
    self.jerk_count += 1
    # Handle cancel state to prevent cruise fault
    if not CS.out.cruiseState.enabled or CS.out.gasPressed or CS.out.brakePressed:
      self.accel_last_jerk = 0.0
      self.jerk = 0.0
      self.jerk_count = 0.0
      self.jerk_upper_limit = 0.0
      self.jerk_lower_limit = 0.0
      self.cb_upper = self.cb_lower = 0.0
      return 0.0

    if state == LongCtrlState.stopping:
      stop_factor = np.clip(CS.out.vEgo / 1.0, 0.0, 1.0)
      self.jerk = (0.75 * stop_factor) - (CS.out.aEgo * 0.5)
      return self.jerk
    else:
      current_accel = np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      self.jerk = (current_accel - self.accel_last_jerk) / self.DT_CTRL
      self.accel_last_jerk = current_accel

    # Calculate jerk limits
    if CS.out.vEgo < 3.13 and CS.out.aEgo > 0:
      jerk_max = np.interp(CS.out.vEgo,
                       [0.0, 1.0, 2.0, 3.13, 5.0],
                       [0.85, 1.5, 2.25, 3.5, 5.0])
    else:
      jerk_max = 5.0

    if self.CP.flags & HyundaiFlags.CANFD.value:
      self.jerk_upper_limit = min(max(0.5, self.jerk * 2.0), jerk_max)
      self.jerk_lower_limit = min(max(1.0, -self.jerk * 4.0), jerk_max)
      self.cb_upper = self.cb_lower = 0.0
    else:
      self.jerk_upper_limit = min(max(0.5, self.jerk * 2.0), jerk_max)
      self.jerk_lower_limit = min(max(1.0, -self.jerk * 2.0), jerk_max)
      if CS.out.aEgo >= 0:
        self.cb_upper = self.cb_lower = 0.0
      else:
        self.cb_upper = np.clip(0.15 + CS.out.aEgo * 0.18, 0.0, 1.0)
        self.cb_lower = np.clip(0.15 + CS.out.aEgo * 0.18, 0.0, 1.0)

    return self.jerk

  def handle_cruise_cancel(self, CS):
    """Handle cruise control cancel to prevent faults."""
    if not CS.out.cruiseState.enabled or CS.out.gasPressed or CS.out.brakePressed:
      self.accel_last = 0.0
      self.last_accel = 0.0
      self.brake_ramp = 0.0
      return True
    return False

  def should_delay_cancel(self, CS):
    """Check if cancel button press should be delayed based on recent deceleration."""
    if CS.out.aEgo < -0.1:
      self.last_decel_time = self.DT_CTRL * self.jerk_count
    current_time = self.DT_CTRL * self.jerk_count
    return (current_time - self.last_decel_time) < self.min_cancel_delay and CS.out.aEgo < 0

  def calculate_limited_accel(self, accel, actuators, CS):
    """Adaptive acceleration limiting."""
    if self.handle_cruise_cancel(CS):
      return accel
    self.make_jerk(CS, actuators)

    target_accel = accel

    # Apply transition smoothing when switching modes
    if self.transitioning and self.prev_mode == 'acc' and self.current_mode == 'blended':
      if target_accel < 0 and target_accel < self.accel_last:
        progress = min(1.0, self.mode_transition_timer / self.mode_transition_duration)
        blend_factor = 1.0 - (1.0 - progress) * (1.0 - abs(target_accel / CarControllerParams.ACCEL_MIN))
        target_accel = self.accel_last + (target_accel - self.accel_last) * blend_factor

    # For braking, vary rate based on braking intensity
    if target_accel < 0:
      brake_ratio = np.clip(abs(target_accel / CarControllerParams.ACCEL_MIN), 0.0, 1.0)
      # Gentler for light braking, more responsive for harder braking
      accel_rate_down = self.DT_CTRL * np.interp(brake_ratio,
                                            [0.0, 0.3, 0.7, 1.0],
                                            [1.3, 1.85, 2.5, 3.5])
      accel = max(target_accel, self.accel_last - accel_rate_down)


    # Apply jerk limits
    accel = np.clip(accel, self.accel_last - self.jerk_lower_limit * self.DT_CTRL,
                 self.accel_last + self.jerk_upper_limit * self.DT_CTRL)

    self.accel_last = accel
    return accel

  def calculate_accel(self, accel, actuators, CS):
    """Calculate acceleration with cruise control status handling."""
    if self.handle_cruise_cancel(CS):
      return 0.0
    if CS.out.aEgo <= 0:
      accel = self.calculate_limited_accel(accel, actuators, CS)
      return float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    return float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))


  def apply_tune(self, CP: Any) -> None:
    """Apply tuning to CarControllerParams."""
    if CP.carFingerprint in (CAR.KIA_EV6):
      CP.vEgoStopping = 0.05
    else:
      CP.vEgoStopping = 0.15
    CP.vEgoStarting = 0.1
    CP.stoppingDecelRate = 0.01
    CP.startAccel = 1.6
    CP.startingState = True

  def get_jerk(self) -> JerkOutput:
    return JerkOutput(
      self.jerk_upper_limit,
      self.jerk_lower_limit,
      self.cb_upper,
      self.cb_lower,
    )

  def calculate_and_get_jerk(self, CS, actuators):
    """Calculate jerk and return JerkOutput."""
    if self.hkg_tuning:
      self.make_jerk(CS, actuators)
      return self.get_jerk()


class HKGLongitudinalController:
  def __init__(self, CP):
    self.CP = CP
    self.tuning = HKGLongitudinalTuning(CP) if Params().get_bool("HKGtuning") else None
    self.jerk = None

  def apply_tune(self, CP):
    if self.tuning:
      self.tuning.apply_tune(CP)

  def calculate_normal_jerk(self, CP: Any, long_control_state: LongCtrlState) -> float:
    """Calculate normal jerk based on long control state."""
    # Hyundai Kona EV CAN requires higher jerk limits to stop appropriately
    if CP.carFingerprint in (CAR.HYUNDAI_KONA_EV):
      return 8.0 if long_control_state == LongCtrlState.pid else 6.0
    return 3.0 if long_control_state == LongCtrlState.pid else 1.0
