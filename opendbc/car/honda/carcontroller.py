import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, rate_limit, make_tester_present_msg, structs
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import CAR, CruiseButtons, HONDA_BOSCH, HONDA_BOSCH_CANFD, HONDA_BOSCH_RADARLESS, \
                                     HONDA_BOSCH_TJA_CONTROL, HONDA_ELESYS, HONDA_NIDEC_ALT_PCM_ACCEL, CarControllerParams
from opendbc.car.interfaces import CarControllerBase

from opendbc.sunnypilot.car.honda.mads import MadsCarController
from opendbc.sunnypilot.car.honda.gas_interceptor import GasInterceptorCarController
from opendbc.sunnypilot.car.honda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


def compute_gb_honda_bosch(accel, speed):
  # TODO returns 0s, is unused
  return 0.0, 0.0


def compute_gb_honda_nidec(accel, speed):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(accel) / 4.8 - creep_brake
  return np.clip(gb, 0.0, 1.0), np.clip(-gb, 0.0, 1.0)


# FORK(HONDA_ELESYS): measured constants for Elesys-radar cars -- see redlight_overshoot_findings.md
# Brake gain: m/s^2 of decel at full COMPUTER_BRAKE (256), fitted over ~45 drives; the ILX-derived
# upstream mapping assumes 4.8 for both sides, but these cars only reach ~2.6 on the brake.
ELESYS_GAS_SCALE = 4.8  # gas keeps upstream scaling; the interceptor gas_mult handles speed shaping
ELESYS_BRAKE_SCALE = 2.6
# Measured self-propulsion (torque-converter creep) on flat ground, m/s^2 vs speed.
# Grade-corrected coastdown pooled over 76 local routes (~140k low-speed coast frames), consistent
# with the single new-build route ac35d9891f: ~1.05 @ 0.3-0.6 m/s, 0.79 @ 0.6-1, 0.66 @ 1-1.5,
# 0.43 @ 1.5-2, ~0.3 @ 2-4, ~0 by 5.
ELESYS_CREEP_BP = [0., 0.75, 1.75, 3.0, 5.0]
ELESYS_CREEP_V = [1.15, 0.8, 0.45, 0.3, 0.0]


def compute_gb_honda_elesys(accel, speed):
  #TODO move scaling variable here once done

  net = float(accel) - float(np.interp(speed, ELESYS_CREEP_BP, ELESYS_CREEP_V))
  gas = max(net, 0.) / ELESYS_GAS_SCALE
  brake = max(-net, 0.) / ELESYS_BRAKE_SCALE
  return float(np.clip(gas, 0.0, 1.0)), float(np.clip(brake, 0.0, 1.0))


def compute_gas_brake(accel, speed, fingerprint):
  if fingerprint in HONDA_BOSCH:
    return compute_gb_honda_bosch(accel, speed)
  elif fingerprint in HONDA_ELESYS:
    return compute_gb_honda_elesys(accel, speed)
  else:
    return compute_gb_honda_nidec(accel, speed)


# TODO not clear this does anything useful
def actuator_hysteresis(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params
  brake_hyst_on = 0.02    # to activate brakes exceed this value
  brake_hyst_off = 0.005  # to deactivate brakes below this value
  brake_hyst_gap = 0.01   # don't change brake command for small oscillations within this value

  # *** hysteresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  return brake, braking, brake_steady


def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_ts, ts, steady_refresh=20.):
  pump_on = False

  # reset pump timer if:
  # - there is an increment in brake request
  # - we are applying steady state brakes and we haven't been running the pump
  #   for more than steady_refresh s (to prevent pressure bleeding)
  # FORK(HONDA_ELESYS): stock Elesys re-primes the pump all through braking. Bus-2 pump
  # duty over 19 routes where stock braking reached the car: ~99% while cb rising, ~39% steady,
  # ~10% falling (50% overall); and decel measurably fades when the pump is off at steady
  # command (+0.04 m/s^3) vs building when on (-0.07). Upstream's 20 s refresh lets pressure
  # bleed through an entire stop approach -- the "loses brake pressure" overshoot. A 0.5 s
  # refresh (0.2 s on / 0.5 s period = 40% steady duty) matches the stock envelope.
  if apply_brake > apply_brake_last or (ts - last_pump_ts > steady_refresh and apply_brake > 0):
    last_pump_ts = ts

  # once the pump is on, run it for at least 0.2s
  if ts - last_pump_ts < 0.2 and apply_brake > 0:
    pump_on = True

  return pump_on, last_pump_ts


def process_hud_alert(hud_alert):
  alert_fcw = False
  alert_steer_required = False

  # Make sure FCW is prioritized over steering required
  # TODO: implement separate available LDW alert
  if hud_alert == VisualAlert.fcw:
    alert_fcw = True
  elif hud_alert in (VisualAlert.steerRequired, VisualAlert.ldw):
    alert_steer_required = True

  return alert_fcw, alert_steer_required


class CarController(CarControllerBase, MadsCarController, GasInterceptorCarController, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    MadsCarController.__init__(self)
    GasInterceptorCarController.__init__(self, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)
    self.CAN = hondacan.CanBus(CP)
    self.tja_control = CP.carFingerprint in HONDA_BOSCH_TJA_CONTROL

    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.stopping_counter = 0

    self.accel = 0.0
    self.speed = 0.0
    self.gas = 0.0
    self.brake = 0.0
    self.last_torque = 0.0

  def update(self, CC, CC_SP, CS, now_nanos):
    MadsCarController.update(self, self.CP, CC, CC_SP)
    actuators = CC.actuators
    hud_control = CC.hudControl
    hud_v_cruise = hud_control.setSpeed / CS.v_cruise_factor if hud_control.speedVisible else 255
    pcm_cancel_cmd = CC.cruiseControl.cancel

    if CC.longActive:
      accel = actuators.accel
      gas, brake = compute_gas_brake(actuators.accel, CS.out.vEgo, self.CP.carFingerprint)
    else:
      accel = 0.0
      gas, brake = 0.0, 0.0

    # *** rate limit steer ***
    limited_torque = rate_limit(actuators.torque, self.last_torque, -self.params.STEER_DELTA_DOWN * DT_CTRL,
                                self.params.STEER_DELTA_UP * DT_CTRL)
    self.last_torque = limited_torque

    # *** apply brake hysteresis ***
    pre_limit_brake, self.braking, self.brake_steady = actuator_hysteresis(brake, self.braking, self.brake_steady,
                                                                           CS.out.vEgo, self.CP.carFingerprint)

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(pre_limit_brake, self.brake_last, -2., DT_CTRL)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    alert_fcw, alert_steer_required = process_hud_alert(hud_control.visualAlert)

    # **** process the car messages ****

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_torque = int(np.interp(-limited_torque * self.params.STEER_MAX,
                                 self.params.STEER_LOOKUP_BP, self.params.STEER_LOOKUP_V))

    # Send CAN commands
    can_sends = []

    # tester present - w/ no response (keeps radar disabled)
    if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS) and self.CP.openpilotLongitudinalControl:
      if self.frame % 10 == 0:
        can_sends.append(make_tester_present_msg(0x18DAB0F1, 1, suppress_response=True))

    # Send steering command.
    can_sends.append(hondacan.create_steering_control(self.packer, self.CAN, apply_torque, CC.latActive, self.tja_control))

    # wind brake from air resistance decel at high speed
    wind_brake = np.interp(CS.out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15])
    # all of this is only relevant for HONDA NIDEC
    max_accel = np.interp(CS.out.vEgo, self.params.NIDEC_MAX_ACCEL_BP, self.params.NIDEC_MAX_ACCEL_V)
    # TODO this 1.44 is just to maintain previous behavior
    pcm_speed_BP = [-wind_brake,
                    -wind_brake * (3 / 4),
                    0.0,
                    0.5]
    # The Honda ODYSSEY seems to have different PCM_ACCEL
    # msgs, is it other cars too?
    if self.CP_SP.enableGasInterceptor or not CC.longActive:
      pcm_speed = 0.0
      pcm_accel = int(0.0)
    elif self.CP.carFingerprint in HONDA_NIDEC_ALT_PCM_ACCEL:
      pcm_speed_V = [0.0,
                     np.clip(CS.out.vEgo - 3.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 0.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 5.0, 0.0, 100.0)]
      pcm_speed = float(np.interp(gas - brake, pcm_speed_BP, pcm_speed_V))
      pcm_accel = int(1.0 * self.params.NIDEC_GAS_MAX)
    else:
      pcm_speed_V = [0.0,
                     np.clip(CS.out.vEgo - 2.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 2.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 5.0, 0.0, 100.0)]
      pcm_speed = float(np.interp(gas - brake, pcm_speed_BP, pcm_speed_V))
      pcm_accel = int(np.clip((accel / 1.44) / max_accel, 0.0, 1.0) * self.params.NIDEC_GAS_MAX)

    if not self.CP.openpilotLongitudinalControl:
      if self.frame % 2 == 0 and self.CP.carFingerprint not in HONDA_BOSCH_RADARLESS | HONDA_BOSCH_CANFD:
        can_sends.append(hondacan.create_bosch_supplemental_1(self.packer, self.CAN))
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.CANCEL, self.CP.carFingerprint))
      elif CC.cruiseControl.resume:
        can_sends.append(hondacan.spam_buttons_command(self.packer, self.CAN, CruiseButtons.RES_ACCEL, self.CP.carFingerprint))

    else:
      # Send gas and brake commands.
      if self.frame % 2 == 0:
        ts = self.frame * DT_CTRL

        if self.CP.carFingerprint in HONDA_BOSCH:
          self.accel = float(np.clip(accel, self.params.BOSCH_ACCEL_MIN, self.params.BOSCH_ACCEL_MAX))
          self.gas = float(np.interp(accel, self.params.BOSCH_GAS_LOOKUP_BP, self.params.BOSCH_GAS_LOOKUP_V))

          stopping = actuators.longControlState == LongCtrlState.stopping
          self.stopping_counter = self.stopping_counter + 1 if stopping else 0
          can_sends.extend(hondacan.create_acc_commands(self.packer, self.CAN, CC.enabled, CC.longActive, self.accel, self.gas,
                                                        self.stopping_counter, self.CP.carFingerprint))
        else:
          apply_brake = np.clip(self.brake_last - wind_brake, 0.0, 1.0)
          apply_brake = int(np.clip(apply_brake * self.params.NIDEC_BRAKE_MAX, 0, self.params.NIDEC_BRAKE_MAX - 1))
          steady_refresh = 0.5 if self.CP.carFingerprint in HONDA_ELESYS else 20.  # FORK: see brake_pump_hysteresis
          pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts, steady_refresh)

          pcm_override = True
          can_sends.append(hondacan.create_brake_command(self.packer, self.CAN, apply_brake, pump_on,
                                                         pcm_override, pcm_cancel_cmd, alert_fcw,
                                                         self.CP.carFingerprint, CS.stock_brake, CS.is_metric, self.CP_SP))
          self.apply_brake_last = apply_brake
          self.brake = apply_brake / self.params.NIDEC_BRAKE_MAX

          can_sends.extend(GasInterceptorCarController.update(self, CC, CS, gas, brake, wind_brake, self.packer, self.frame))

    # Stock ACC stand-down: the Elesys radar keeps its own ACC armed off the PCM cruise mirror, so
    # clearing cruise buttons never disengaged it. It does read the master ACC on/off (MAIN) switch
    # from SCM_BUTTONS, so we re-send SCM_BUTTONS to it (bus 2) with MAIN_ON=0 -> the stock ACC stands
    # down, stopping the blocked ACC brake that trips the VSA TSA fault. The PCM on the pt bus still
    # gets the driver's real MAIN/buttons (OP engages normally); CMBS/FCW are independent of MAIN. ~25 Hz.
    if self.CP.carFingerprint in HONDA_ELESYS and self.CP.openpilotLongitudinalControl and self.frame % 4 == 0:
      can_sends.append(hondacan.create_scm_buttons_no_cruise(self.packer, self.CAN.camera, CS.scm_buttons))

    # Send dashboard UI commands.
    if self.frame % 10 == 0:
      if self.CP.openpilotLongitudinalControl:
        # On Nidec, this also controls longitudinal positive acceleration
        can_sends.append(hondacan.create_acc_hud(self.packer, self.CAN.pt, self.CP, CC.enabled, pcm_speed, pcm_accel,
                                                 hud_control, hud_v_cruise, CS.is_metric, CS.acc_hud))

      steering_available = CS.out.cruiseState.available and CS.out.vEgo > max(self.params.STEER_GLOBAL_MIN_SPEED, self.CP.minSteerSpeed)
      # HONDA_ELESYS: 4-byte LKAS_HUD with a different layout; stock camera's HUD is forwarded instead
      if self.CP.carFingerprint not in HONDA_ELESYS:
        can_sends.extend(hondacan.create_lkas_hud(self.packer, self.CAN.lkas, self.CP, hud_control, CC.latActive,
                                                  steering_available, alert_steer_required, CS.lkas_hud, self.dashed_lanes))

      if self.CP.openpilotLongitudinalControl:
        # TODO: combining with create_acc_hud block above will change message order and will need replay logs regenerated
        if self.CP.carFingerprint in (HONDA_BOSCH - HONDA_BOSCH_RADARLESS):
          can_sends.append(hondacan.create_radar_hud(self.packer, self.CAN.pt))
        if self.CP.carFingerprint == CAR.HONDA_CIVIC_BOSCH:
          can_sends.append(hondacan.create_legacy_brake_command(self.packer, self.CAN.pt))
        if self.CP.carFingerprint not in HONDA_BOSCH:
          self.speed = pcm_speed
          if not self.CP_SP.enableGasInterceptor:
            self.gas = pcm_accel / self.params.NIDEC_GAS_MAX

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, self.packer, self.frame,
                                                                       self.last_button_frame, self.CAN))

    new_actuators = actuators.as_builder()
    new_actuators.speed = self.speed
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas
    new_actuators.brake = self.brake
    new_actuators.torque = self.last_torque
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
