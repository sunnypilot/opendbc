import math
import numpy as np
from opendbc.car.carlog import carlog
from opendbc.car.vehicle_model import VehicleModel

try:
  # TODO-SP: We shouldn't really import params from here, but it's the easiest way to get the params for
  #  live tuning temporarily while we understand the angle steering better
  from openpilot.common.params import Params
  PARAMS_AVAILABLE = True
except ImportError:
  carlog.warning("Unable to import Params from openpilot.common.params.")
  PARAMS_AVAILABLE = False

from opendbc.can.packer import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_driver_steer_torque_limits, common_fault_avoidance, \
  make_tester_present_msg, structs, AngleSteeringLimits, rate_limit
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase, ISO_LATERAL_ACCEL

from opendbc.sunnypilot.car.hyundai.escc import EsccCarController
from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalController
from opendbc.sunnypilot.car.hyundai.mads import MadsCarController

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_FAULT_ANGLE = 85
MAX_FAULT_ANGLE_FRAMES = 89
MAX_FAULT_ANGLE_CONSECUTIVE_FRAMES = 2


class LKASTorqueCalculator:
  def __init__(self, freq_hz=50.):
    # Global parameters
    self.min_torque = CarControllerParams.ANGLE_MIN_TORQUE
    self.max_torque = CarControllerParams.ANGLE_MAX_TORQUE
    self.max_from_speed = 96
    self.rate_limit = 500
    self.la_deadzone = 0.38

    # Control gains
    self.k1 = 200  # Lateral acceleration gain
    self.k2 = 20  # Lateral jerk gain
    self.k3 = 1.0  # Lateral acceleration power
    self.k4 = 1  # Lateral jerk power
    self.k5 = 10  # Speed feedforward gain

    # State variable for rate limiting
    self.old_output = 0

    # Simple history for derivative (just last 2 values)
    self.lat_accel_history = np.array([0.0, 0.0])  # [previous, current]

    # Time step (assuming fixed frequency control loop)
    self.dt = 1./freq_hz  # Default 50Hz (0.02s)

  def sign(self, number):
    """Return the sign of a number"""
    if number > 0:
      return 1
    elif number == 0:
      return 0
    else:
      return -1

  def apply_rate_limit(self, old_val, new_val, limit):
    """Apply rate limiting to prevent sudden changes"""
    return min(max(new_val, old_val - limit), old_val + limit)

  def apply_deadzone(self, val, deadzone):
    """Apply deadzone to eliminate small values/noise"""
    if abs(val) <= deadzone:
      return 0.0
    elif val < 0.0:
      return val + deadzone
    else:
      return val - deadzone

  def calculate_derivative(self, current_value):
    """
    Calculate derivative (lateral jerk) using numpy for clean history management

    Args:
        current_value: Current desired lateral acceleration

    Returns:
        float: Derivative (lateral jerk)
    """
    # Shift history: [old_previous, old_current] -> [old_current, new_current]
    self.lat_accel_history = np.roll(self.lat_accel_history, -1)
    self.lat_accel_history[-1] = current_value

    # Calculate derivative: (current - previous) / dt
    derivative = (self.lat_accel_history[1] - self.lat_accel_history[0]) / self.dt

    return derivative

  def calculate_torque(self, desired_lat_accel, ego_velocity, lkas_active: bool, min_active_torque=None):
    """
    Calculate LKAS torque command in real-time

    Args:
        desired_lat_accel: Desired lateral acceleration (v2)
        ego_velocity: Vehicle ego velocity (v4)
        lkas_active: LKAS active flag (v5) - 1.0 means system disabled
        max_from_speed: Optional maximum torque from speed (v6) - if not provided, uses self.max_from_speed

    Returns:
        float: Calculated torque command (0-250)
    """
    # Calculate lateral jerk as derivative of lateral acceleration
    desired_lat_jerk = self.calculate_derivative(desired_lat_accel)

    # Apply deadzone to lateral acceleration
    la = self.apply_deadzone(desired_lat_accel, self.la_deadzone)
    lj = desired_lat_jerk

    # If lateral acceleration is zero (in deadzone), zero out jerk too
    if la == 0.0:
      lj = 0.0

    # Calculate lateral acceleration component
    fla = min(abs(self.k1 * la) ** self.k3, self.max_torque)

    # Calculate lateral jerk component
    flj = min(abs(self.k2 * lj) ** self.k4, self.max_torque)

    # Start with lateral acceleration component
    out = fla

    # Add speed feedforward component
    flv = min(min_active_torque or self.max_from_speed, self.k5 * ego_velocity)
    out = out + flv

    # Apply initial bounds
    out = max(min(out, self.max_torque), self.min_torque)

    # Apply jerk component based on sign relationship
    if self.sign(la) == self.sign(lj):
      out = out - flj  # Same sign: subtract (damping)
    else:
      out = out + flj  # Opposite sign: add (enhancement)

    # System disable check
    if not lkas_active:
      out = 0.0

    # Apply final bounds
    out = max(min(out, self.max_torque), self.min_torque)

    # Apply rate limiting
    out = self.apply_rate_limit(self.old_output, out, self.rate_limit)

    # Update state for next iteration
    self.old_output = out

    return out


MAX_ANGLE_RATE = 5
# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~3.6 m/s^2
MAX_LATERAL_JERK = 3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~3.6 m/s^3

def get_max_angle_rate_sec(v_ego_raw: float, VM: VehicleModel):
  max_curvature_rate_sec = MAX_LATERAL_JERK / (v_ego_raw ** 2)  # (1/m)/s
  max_angle_rate_sec = math.degrees(VM.get_steer_from_curvature(max_curvature_rate_sec, v_ego_raw, 0))  # deg/s
  return max_angle_rate_sec

def get_max_angle_delta(v_ego_raw: float, VM: VehicleModel, freq=100.):
  return get_max_angle_rate_sec(v_ego_raw, VM) / float(freq) # hz

def get_max_angle(v_ego_raw: float, VM: VehicleModel):
  max_curvature = MAX_LATERAL_ACCEL / (v_ego_raw ** 2)  # 1/m
  return math.degrees(VM.get_steer_from_curvature(max_curvature, v_ego_raw, 0))  # deg

def apply_hyundai_steer_angle_limits(apply_angle: float, apply_angle_last: float, v_ego_raw: float, steering_angle: float,
                                     lat_active: bool, limits: AngleSteeringLimits, VM: VehicleModel, smoothing_factor) -> float:
  new_angle = np.clip(apply_angle, -819.2, 819.1)
  v_ego_raw = max(v_ego_raw, 1)

  if abs(new_angle - apply_angle_last) > 0.1:  # If there's a significant difference between the new angle and the last applied angle, apply smoothing
    adjusted_alpha = np.interp(v_ego_raw, CarControllerParams.SMOOTHING_ANGLE_VEGO_MATRIX, CarControllerParams.SMOOTHING_ANGLE_ALPHA_MATRIX) + smoothing_factor
    adjusted_alpha_limited = float(min(float(adjusted_alpha), 1.))  # Limit the smoothing factor to 1 if adjusted_alpha is greater than 1
    new_angle = (new_angle * adjusted_alpha_limited) + (apply_angle_last * (1 - adjusted_alpha_limited))

  apply_angle = new_angle

  # *** max lateral jerk limit ***
  max_angle_delta = get_max_angle_delta(v_ego_raw, VM)

  # prevent fault
  max_angle_delta = min(max_angle_delta, MAX_ANGLE_RATE)
  new_apply_angle = rate_limit(apply_angle, apply_angle_last, -max_angle_delta, max_angle_delta)

  # *** max lateral accel limit ***
  max_angle = get_max_angle(v_ego_raw, VM)
  new_apply_angle = np.clip(new_apply_angle, -max_angle, max_angle)

  # angle is current angle when inactive
  if not lat_active:
    new_apply_angle = steering_angle

  # prevent fault
  return float(np.clip(new_apply_angle, -limits.STEER_ANGLE_MAX, limits.STEER_ANGLE_MAX))

def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning

def get_safety_CP():
  from opendbc.car.hyundai.interface import CarInterface
  return CarInterface.get_non_essential_params("HYUNDAI_IONIQ_5_PE")


class CarController(CarControllerBase, EsccCarController, LongitudinalController, MadsCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    EsccCarController.__init__(self, CP, CP_SP)
    MadsCarController.__init__(self)
    LongitudinalController.__init__(self, CP, CP_SP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.car_fingerprint = CP.carFingerprint
    self.calc = LKASTorqueCalculator(freq_hz=100)

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

    self.accel_last = 0
    self.apply_torque_last = 0
    self.apply_angle_last = 0
    self.lkas_max_torque = 0
    self.last_button_frame = 0
    self.angle_limit_counter = 0
    self.smoothing_factor = 0.6
    self.last_override_frame = 0

    self.angle_min_active_torque = self.params.ANGLE_MIN_TORQUE
    self.angle_max_torque = self.params.ANGLE_MAX_TORQUE
    self.angle_torque_override_cycles = self.params.ANGLE_TORQUE_OVERRIDE_CYCLES
    self._params = Params() if PARAMS_AVAILABLE else None
    if PARAMS_AVAILABLE:
      # self.live_tuning = self._params.get_bool("HkgAngleLiveTuning")
      self.smoothing_factor = float(self._params.get("HkgTuningAngleSmoothingFactor")) / 10.0 if self._params.get("HkgTuningAngleSmoothingFactor") else 0.0
      self.angle_min_active_torque = int(self._params.get("HkgTuningAngleMinTorque")) if self._params.get("HkgTuningAngleMinTorque") else 0
      self.angle_max_torque = int(self._params.get("HkgTuningAngleMaxTorque")) if self._params.get("HkgTuningAngleMaxTorque") else 0
      self.angle_torque_override_cycles = int(self._params.get("HkgTuningOverridingCycles")) if self._params.get("HkgTuningOverridingCycles") else 0


  def update(self, CC, CC_SP, CS, now_nanos):
    EsccCarController.update(self, CS)
    MadsCarController.update(self, self.CP, CC, CC_SP, self.frame)
    if self.frame % 2 == 0:
      LongitudinalController.update(self, CC, CS)

    actuators = CC.actuators
    hud_control = CC.hudControl
    apply_torque = 0

    # if PARAMS_AVAILABLE and self.live_tuning and self._params and self.frame % 500 == 0:
    #   if (smoothingFactorParam := self._params.get("HkgTuningAngleSmoothingFactor")) and float(smoothingFactorParam) != self.smoothing_factor:
    #     self.smoothing_factor = float(smoothingFactorParam) / 10.0
    #   if (minTorqueParam := self._params.get("HkgTuningAngleMinTorque")) and int(minTorqueParam) != self.angle_min_active_torque:
    #     self.angle_min_active_torque = int(minTorqueParam)
    #   if (maxTorqueParam := self._params.get("HkgTuningAngleMaxTorque")) and int(maxTorqueParam) != self.angle_max_torque:
    #     self.angle_max_torque = int(maxTorqueParam)
    #   if (overrideCyclesParam := self._params.get("HkgTuningOverridingCycles")) and int(overrideCyclesParam) != self.angle_torque_override_cycles:
    #     self.angle_torque_override_cycles = int(overrideCyclesParam)

    # TODO: needed for angle control cars?
    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_FAULT_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_FAULT_ANGLE_FRAMES,
                                                                       MAX_FAULT_ANGLE_CONSECUTIVE_FRAMES)

    # steering torque
    if not self.CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING:
      new_torque = int(round(actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

    # angle control
    else:
      steering_active = CC.latActive and not CS.out.steeringPressed
      self.apply_angle_last = apply_hyundai_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                               CS.out.steeringAngleDeg, steering_active,
                                                               CarControllerParams.ANGLE_LIMITS, self.VM, self.smoothing_factor)
      if CS.out.steeringPressed:  # User is overriding
        # Let's try to consider that the override is not a true or false but a progressive depending on how much torque is being applied to the col
        self.last_override_frame = self.frame
        target_torque = self.params.ANGLE_MIN_TORQUE
        torque_delta = self.lkas_max_torque - target_torque
        adaptive_ramp_rate = max(torque_delta / self.angle_torque_override_cycles, 1)  # Ensure at least 1 unit per cycle
        self.lkas_max_torque = max(self.lkas_max_torque - adaptive_ramp_rate, self.params.ANGLE_MIN_TORQUE)
      else:
        target_torque = self.calc.calculate_torque(
          desired_lat_accel=actuators.curvature * CS.out.vEgoRaw ** 2,
          ego_velocity=CS.out.vEgoRaw,
          lkas_active=CC.latActive,
          min_active_torque=self.angle_min_active_torque
        )

        # Ramp up or down toward the target torque smoothly
        if self.lkas_max_torque > target_torque:
          self.lkas_max_torque = max(self.lkas_max_torque - self.params.ANGLE_RAMP_DOWN_RATE, target_torque)
        else:
          self.lkas_max_torque = min(self.lkas_max_torque + self.params.ANGLE_RAMP_UP_RATE, target_torque)

      # Safety clamp
      self.lkas_max_torque = float(np.clip(self.lkas_max_torque, self.params.ANGLE_MIN_TORQUE, self.angle_max_torque))

    if not CC.latActive:
      apply_torque = 0
      self.lkas_max_torque = 0

    # Hold torque with induced temporary fault when cutting the actuation bit
    # FIXME: we don't use this with CAN FD?
    torque_fault = CC.latActive and not apply_steer_req

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not ((self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) or self.ESCC.enabled) and \
            self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC))
    else:
      can_sends.extend(self.create_can_msgs(apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel,
                                            stopping, hud_control, actuators, CS, CC))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.accel = self.tuning.actual_accel

    self.frame += 1
    return new_actuators, can_sends

  def create_can_msgs(self, apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel, stopping, hud_control, actuators, CS, CC):
    can_sends = []

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                              torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                              hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                              left_lane_warning, right_lane_warning,
                                              self.lkas_icon))

    # Button messages
    if not self.CP.openpilotLongitudinalControl:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame

    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
      # TODO: unclear if this is needed
      jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, self.CP,
                                                      CS.main_cruise_enabled, self.tuning, self.ESCC))

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled, self.lfa_icon))

    # 5 Hz ACC options
    if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP, self.ESCC))

    # 2 Hz front radar options
    if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl and not self.ESCC.enabled:
      can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    return can_sends

  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC):
    can_sends = []

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque,
                                                           self.apply_angle_last, self.lkas_max_torque, self.lkas_icon))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    if self.frame % 5 == 0 and lka_steering:
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT))

    # LFA and HDA icons
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled, self.lfa_icon))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      else:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control, CS.main_cruise_enabled, self.tuning))
        self.accel_last = accel
    else:
      # button presses
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends
