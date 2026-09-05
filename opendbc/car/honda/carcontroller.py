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
from opendbc.sunnypilot.car.honda.dynamic_tuning import HondaDynamicTuner

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


def compute_gb_honda_elesys(accel, speed):
  creep = float(np.interp(speed, [0., 0.75, 1.75, 3.0, 5.0], [1.15, 0.8, 0.45, 0.3, 0.0]))
  creep *= float(np.clip(1. - max(float(accel), 0.) / 0.8, 0., 1.))
  net = float(accel) - creep
  gas = max(net, 0.) / 4.8
  brake = max(-net, 0.) / 2.6
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


def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_ts, ts):
  pump_on = False

  # reset pump timer if:
  # - there is an increment in brake request
  # - we are applying steady state brakes and we haven't been running the pump
  #   for more than 20s (to prevent pressure bleeding)
  if apply_brake > apply_brake_last or (ts - last_pump_ts > 20. and apply_brake > 0):
    last_pump_ts = ts

  # once the pump is on, run it for at least 0.2s
  if ts - last_pump_ts < 0.2 and apply_brake > 0:
    pump_on = True

  return pump_on, last_pump_ts


ELESYS_PUMP_RUN = 0.5          # s of pump run per trigger
ELESYS_PUMP_REFRACTORY = 3.0   # s of enforced quiet after a run before a small rise can re-trigger
# Counts of command rise needed to re-prime, scaled by how hard the brake is already on.
# v5: this was a flat 3. Because `rise and in_run` re-primes INSIDE the current run, a command
# that climbs 3 counts every 0.5 s keeps the pump on continuously -- correct on a real apply
# ramp into a stop, wrong during ordinary gentle deceleration where the command drifts up
# slowly. Measured over 405.9 min engaged / 149.2 min braking on routes 15646e8515eda1a7
# (13 routes), a flat deadband of 3 put 48.2% of all moving pump run time into cb < 60, at a
# local duty of 0.27-0.37 -- i.e. at light braking the pump was running a third of the time.
# That is the audible "wind when it isn't needed"; the periodic backstop is NOT the cause
# (stretching it 12 -> 20 s saves 0.3 min of 38.0 -- and costs dearly elsewhere, see the
# MOVE_REFRESH note below; do not do it). Requiring a bigger rise to re-prime at low
# command cuts light-braking run time 15.75 -> 12.10 min (-23%) and total 38.0 -> 33.3 min
# (-12%) for +7.8% starts, with the worst pump-off gap while moving UNCHANGED at 8.40 s
# (accel < -1.0) and 5.88 s (accel < -1.5). Firm braking keeps the old deadband of 3, so
# pressure build where it matters is untouched.
# If it is still too whiny, the next measured steps are (16, 8, 3) -> 11.47 min light / 31.8
# total / gap@-1.5 improves to 5.62 s, then (20, 10, 4) -> 10.77 / 30.2 but gap@-1.5 grows to
# 6.06 s, which is where it starts costing pressure.
ELESYS_PUMP_DEADBAND_BP = [0., 60., 200.]
ELESYS_PUMP_DEADBAND_V = [12., 6., 3.]
ELESYS_PUMP_BIG_RISE = 15      # counts; a real apply ramp, bypasses the refractory
ELESYS_PUMP_HOLD_REFRESH = 30.0        # s top-up period at standstill
ELESYS_PUMP_MOVE_REFRESH_BP = [40., 200.]    # apply_brake counts
ELESYS_PUMP_MOVE_REFRESH_V = [12.0, 6.0]     # s backstop while moving: light braking -> firm
# DO NOT relax the moving backstop to quiet the pump. It carries almost no duty, but it is the
# only thing bounding the worst DRY STRETCH, which is the metric that actually matters for
# pressure. Measured: 12 -> 20 s buys -1.0% pump time and 17 fewer starts, and costs the worst
# pump-off stretch while braking and moving 11.48 -> 18.42 s. Concrete instance on route
# 00000020, t=1662.3-1680.8 s: 922 consecutive frames, no data hole, vEgo 16.2-17.7 m/s, cb mean
# 57 / peak 102, commanded accel to -1.25 m/s^2, pump off throughout -- and the stretch length is
# set exactly by the period (interp(57,[40,200],[12,6]) = 11.36 s predicted vs 11.42 s observed).
# Gating the backstop on command instead of period gives the same 18.42 s.
# NB when evaluating any future pump change, use "longest pump-off stretch while any brake
# command exists and the car is moving" as the safety metric. The often-quoted "worst gap at
# accel < -1.5" is insensitive by construction: at that command the period is ~6 s and only 2 of
# 269 such gaps are ended by a backstop fire, so it reads the same for every variant.


def brake_pump_hysteresis_elesys(apply_brake, v_ego, brake_anchor, last_pump_ts, ts):
  # FORK(HONDA_ELESYS): pressure bleeds with the pump off, so upstream's 20 s refresh under-pumps
  # (the original stop-overshoot bug), but re-priming on every +1 count made the pump stutter
  # (75 starts/min). This is the v4 shape, tuned for PUMP LIFE rather than for burst count.
  #
  # Why not just revert to upstream: replayed over the full 14-route set (405.9 min engaged,
  # 149.2 min with brake commanded), upstream costs 48.55 min of run time across 2302 starts vs
  # this tuning's 38.00 min across 1619 -- i.e. -22% run time AND -53% starts, so it wins on both
  # axes, not just one. Corrected figure: upstream is 29.9 starts per minute of brake-commanded
  # time (49/min in the 2.5-10 m/s band), NOT the 75/min this comment used to claim.
  #
  # CAVEAT on every number in this block: they are open-loop replays over apply_brake traces that
  # were recorded under EARLIER pump tunings (three pump commits landed inside the logging
  # window), so absolute duty does not transfer to the road -- measured on-wire duty is 52.1% of
  # braking time against 34.0% replayed. Only the A/B deltas between variants are meaningful.
  #
  # What changed from v3, and why the stop quality survives it:
  #  - runs shortened 1.0 s -> 0.5 s. Duty is dominated by LIGHT braking, not by stops: 56% of
  #    moving-brake time sits at cb < 60, where wind_brake drift alone keeps nudging the command
  #    past the deadband. Halving the run halves that cost directly.
  #  - both continuous-run branches were dropped in v4 and are now RESTORED. Dropping them was
  #    a mistake and it silently reverted commit 2905e73d1 ("Fixed Pump Blind Spot on Saturated
  #    Braking"). The v4 argument was about FREQUENCY -- the saturated branch covered "only 2.1%
  #    of moving-brake time" -- but the frames it covers are exactly the maximum-demand ones,
  #    and once the command is firm the rise-trigger is structurally dead (there is nothing left
  #    to rise to), so only the 6 s backstop remains. Measured, v3 -> v4 on the real 13-route
  #    command stream:
  #        cb >= 200 (firm/railed): duty 1.00 -> 0.32, worst pump-off gap 0.16 s -> 5.50 s
  #        stop approach v < 2.5  : duty 0.72 -> 0.26, worst gap        7.00 s -> 10.22 s
  #    That is the exact signature 2905e73d1 was written to cure (~3 s gaps at max demand bled
  #    0.5-0.7 m/s^2, achieved/commanded slope 0.59-0.86, then +0.5-1.0 m/s^2 over target into
  #    the stop). The noise complaint that motivated v4 lives at LIGHT braking, which the
  #    graded deadband below handles on its own -- the two are orthogonal, so we keep both.
  #  - refractory 2.5 -> 3.0 s and moving backstop 8/5 s -> 12/6 s. The 2026-07-12 replay showed
  #    the periodic refresh is nearly irrelevant to duty (15 s ~= 2.5 s), so relaxing it costs
  #    almost nothing. It stays LOAD-SCALED rather than flat on purpose: the backstop is what
  #    re-primes a standstill hold the moment the car starts creeping (the 30 s hold period stops
  #    applying as soon as v > 0.15, and an already-expired timer fires immediately). A flat 10 s
  #    would silently lengthen that recovery for a firm hold; 6 s at cb >= 200 keeps it prompt.
  #  - standstill hold unchanged at 30 s top-ups: 32 frames of motion in 46,815 hold frames.
  #
  # Bleed remains self-healing at LIGHT braking: fading pressure -> fading decel -> the PI raises
  # the command past the deadband -> re-prime. That argument does NOT hold once the command is
  # firm, which is why the continuous branches below have to exist: at a railed command there is
  # nothing left to rise to and the self-healing path is dead.
  if (0.15 <= v_ego < 2.5 and apply_brake > 100) or (v_ego >= 2.5 and apply_brake > 200):
    last_pump_ts = ts
    brake_anchor = apply_brake
    return True, brake_anchor, last_pump_ts

  refresh_period = ELESYS_PUMP_HOLD_REFRESH if v_ego < 0.15 else \
    float(np.interp(apply_brake, ELESYS_PUMP_MOVE_REFRESH_BP, ELESYS_PUMP_MOVE_REFRESH_V))
  deadband = float(np.interp(apply_brake, ELESYS_PUMP_DEADBAND_BP, ELESYS_PUMP_DEADBAND_V))
  rise = apply_brake > brake_anchor + deadband
  big_rise = apply_brake > brake_anchor + ELESYS_PUMP_BIG_RISE
  in_run = ts - last_pump_ts < ELESYS_PUMP_RUN
  idle = ts - last_pump_ts > ELESYS_PUMP_RUN + ELESYS_PUMP_REFRACTORY
  if (rise and (in_run or idle or big_rise)) or (ts - last_pump_ts > refresh_period and apply_brake > 0):
    last_pump_ts = ts
    brake_anchor = apply_brake
  if apply_brake < brake_anchor - 6:  # follow real releases down, ignore jitter
    brake_anchor = apply_brake

  pump_on = (ts - last_pump_ts < ELESYS_PUMP_RUN) and apply_brake > 0
  return pump_on, brake_anchor, last_pump_ts


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
    # FORK: self-learning longitudinal tuning. Inert unless HondaDynamicTuningEnabled is set.
    self.dynamic_tuner = HondaDynamicTuner(CP, CP_SP)

    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.pump_brake_anchor = 0
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

    # FORK: pitch feedforward. Returns 0.0 with the dynamic tuner off, so the
    # accel target is untouched by default. Sign matches ford/toyota: nose-up is
    # positive, climbing adds demand.
    hill_accel = self.dynamic_tuner.update_state(CC, CS)

    if CC.longActive:
      accel = actuators.accel
      adjust_accel = accel + hill_accel
      gas, brake = compute_gas_brake(adjust_accel, CS.out.vEgo, self.CP.carFingerprint)
    else:
      accel = 0.0
      adjust_accel = 0.0
      gas, brake = 0.0, 0.0

    # *** rate limit steer ***
    limited_torque = rate_limit(actuators.torque, self.last_torque, -self.params.STEER_DELTA_DOWN * DT_CTRL,
                                self.params.STEER_DELTA_UP * DT_CTRL)
    self.last_torque = limited_torque

    # *** apply brake hysteresis ***
    pre_limit_brake, self.braking, self.brake_steady = actuator_hysteresis(brake, self.braking, self.brake_steady,
                                                                           CS.out.vEgo, self.CP.carFingerprint)

    # *** rate limit after the enable check ***
    # NB: MVL runs a 3x faster brake rise here (3 * DT_CTRL). Deliberately NOT ported: combined
    # with the learned brake gain it reaches full brake authority from a gentle request in about
    # a tenth of a second. The two multiply, so only one of them can be loosened at a time.
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
    # same aero term expressed in m/s^2, used by the dynamic tuner's gas path
    wind_brake_ms2 = np.interp(CS.out.vEgo, [0.0, 13.4, 22.4, 31.3, 40.2], [0.000, 0.049, 0.136, 0.267, 0.441])
    # all of this is only relevant for HONDA NIDEC
    max_accel = np.interp(CS.out.vEgo, self.params.NIDEC_MAX_ACCEL_BP, self.params.NIDEC_MAX_ACCEL_V)
    # TODO this 1.44 is just to maintain previous behavior
    pcm_speed_BP = [-wind_brake,
                    -wind_brake * (3 / 4),
                    0.0,
                    0.5]
    # The Honda ODYSSEY seems to have different PCM_ACCEL
    # msgs, is it other cars too?
    #
    # FORK: an earlier revision crossfaded part of the gas request back to the PCM above
    # ~30 km/h. That is gone. The PCM was never once shown to answer: across 17 engaged
    # routes openpilot sent 289,625 ACC_HUD frames with PCM_GAS = 0 and PCM_SPEED = 0 in
    # 100% of them, so the whole channel was inherited belief, never a measurement. The
    # interceptor is also simply the easier actuator to control -- the PCM has its own
    # limits, lags and dual-servo behaviour, and it differs per car, so anything learned
    # about it would not transfer. The interceptor owns the gas at every speed, which is
    # what stock upstream already does for a GasInterceptor car.
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
      pcm_accel = int(np.clip((adjust_accel / 1.44) / max_accel, 0.0, 1.0) * self.params.NIDEC_GAS_MAX)

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
          apply_brake = np.clip(self.brake_last - wind_brake * self.dynamic_tuner.wind_scale(), 0.0, 1.0)
          # FORK: learned brake gain. This is what replaces hand-editing the brake divisor in
          # compute_gb_honda_elesys(); returns 1.0 with the dynamic tuner off.
          brake_gain = self.dynamic_tuner.brake_gain(CC, CS, float(apply_brake))
          apply_brake = int(np.clip(apply_brake * brake_gain * self.params.NIDEC_BRAKE_MAX,
                                    0, self.params.NIDEC_BRAKE_MAX - 1))
          # FORK: limit brake release to 32 counts/frame to match factory -- this is what stops
          # the lurch as it lets go at a stop. Applied before the pump hysteresis so the ELESYS
          # anchor tracks exactly what goes on the wire. Bypassed on disengage or driver
          # override so nothing delays getting off the brakes.
          if self.dynamic_tuner.enabled and CC.longActive and not CS.out.gasPressed and not CS.out.brakePressed:
            apply_brake = max(self.apply_brake_last - 32, apply_brake)
          if self.CP.carFingerprint in HONDA_ELESYS:
            pump_on, self.pump_brake_anchor, self.last_pump_ts = brake_pump_hysteresis_elesys(
              apply_brake, CS.out.vEgo, self.pump_brake_anchor, self.last_pump_ts, ts)
          else:
            pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)

          pcm_override = True
          can_sends.append(hondacan.create_brake_command(self.packer, self.CAN, apply_brake, pump_on,
                                                         pcm_override, pcm_cancel_cmd, alert_fcw,
                                                         self.CP.carFingerprint, CS.stock_brake, CS.is_metric, self.CP_SP))
          self.apply_brake_last = apply_brake
          self.brake = apply_brake / self.params.NIDEC_BRAKE_MAX

          # the learned aero scale has to apply to both sides of the term or the wind learner
          # would be measuring an error it cannot influence
          can_sends.extend(GasInterceptorCarController.update(self, CC, CS, gas, brake,
                                                              wind_brake * self.dynamic_tuner.wind_scale(),
                                                              self.packer, self.frame, self.dynamic_tuner))
          self.dynamic_tuner.update_wind(CC, CS, float(wind_brake_ms2))

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
      else:
        # The stock camera keeps 0x33D. This is the side channel an in-line module reads to
        # merge openpilot's alerts into that frame -- see _sunnypilot_hud.dbc for why it is
        # done this way round rather than by taking 0x33D over.
        can_sends.append(hondacan.create_sp_hud_status(self.packer, self.CAN.camera, CC, hud_control,
                                                       alert_steer_required, alert_fcw))

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

    self.dynamic_tuner.persist(self.frame)
    self.dynamic_tuner.log_state(self.frame)

    self.frame += 1
    return new_actuators, can_sends
