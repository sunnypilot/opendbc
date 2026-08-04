import unittest

import numpy as np

from types import SimpleNamespace

from opendbc.car import Bus
from opendbc.car.honda import hondacan
from opendbc.car.honda.carcontroller import (brake_pump_hysteresis, brake_pump_hysteresis_elesys,
                                             compute_gas_brake, compute_gb_honda_elesys,
                                             compute_gb_honda_nidec)
from opendbc.sunnypilot.car.honda.gas_interceptor import elesys_gas_multiplier

# golden values measured for the 9G Accord AU (redlight_overshoot_findings.md) -- deliberately
# duplicated here so any change to the deployed mapping fails a test until re-measured
GOLD_BRAKE_SCALE = 2.6                          # m/s^2 at full COMPUTER_BRAKE
GOLD_GAS_SCALE = 4.8
GOLD_CREEP_BP = [0., 0.75, 1.75, 3.0, 5.0]      # grade-corrected coastdown, 76 routes
GOLD_CREEP_V = [1.15, 0.8, 0.45, 0.3, 0.0]
from opendbc.car.honda.values import CAR, DBC, HONDA_BOSCH, HONDA_ELESYS

ELESYS_CAR = CAR.HONDA_ACCORD_9G_AU
NIDEC_CAR = CAR.HONDA_CRV  # plain Nidec, no interceptor assumptions in these unit tests


class TestElesysCategory(unittest.TestCase):
  def test_membership(self):
    self.assertTrue(ELESYS_CAR in HONDA_ELESYS)
    self.assertFalse(ELESYS_CAR in HONDA_BOSCH)
    self.assertFalse(NIDEC_CAR in HONDA_ELESYS)
    # every Elesys car needs a brake scale (or falls back to the default) and must not be Bosch
    for car in HONDA_ELESYS:
      self.assertFalse(car in HONDA_BOSCH)

  def test_dispatch(self):
    # Elesys car routes to the Elesys mapping, Nidec car to upstream mapping
    a, v = -1.5, 7.0
    self.assertEqual(compute_gas_brake(a, v, ELESYS_CAR), compute_gb_honda_elesys(a, v))
    self.assertEqual(compute_gas_brake(a, v, NIDEC_CAR), tuple(compute_gb_honda_nidec(a, v)))


class TestComputeGbNidec(unittest.TestCase):
  """The upstream ILX-fitted mapping must be exactly upstream again (no fork leakage)."""

  def test_upstream_formula(self):
    for accel in (-3.0, -1.0, -0.3, 0.0, 0.5, 2.0):
      for speed in (0.0, 1.0, 2.2, 2.3, 10.0):
        creep = max(0.0, (2.3 - speed) / 2.3 * 0.15) if speed < 2.3 else 0.0
        gb = accel / 4.8 - creep
        gas, brake = compute_gb_honda_nidec(accel, speed)
        self.assertAlmostEqual(gas, float(np.clip(gb, 0.0, 1.0)), places=10)
        self.assertAlmostEqual(brake, float(np.clip(-gb, 0.0, 1.0)), places=10)

  def test_signature_is_upstream(self):
    import inspect
    self.assertEqual(list(inspect.signature(compute_gb_honda_nidec).parameters), ['accel', 'speed'])


class TestComputeGbElesys(unittest.TestCase):
  def test_creep_offset_near_stop(self):
    # measured: a small planner decel near standstill must still command a real brake,
    # because the car self-propels at ~1 m/s^2 (torque-converter creep)
    gas, brake = compute_gb_honda_elesys(-0.29, 0.65)
    self.assertEqual(gas, 0.0)
    self.assertGreater(brake * 256, 100)  # above the measured effectiveness threshold

  def test_mid_speed_brake_scale(self):
    # above the creep band the mapping is a pure rescale: accel/2.6
    gas, brake = compute_gb_honda_elesys(-1.5, 7.0)
    self.assertEqual(gas, 0.0)
    self.assertAlmostEqual(brake, 1.5 / GOLD_BRAKE_SCALE, places=6)

  def test_gas_side(self):
    gas, brake = compute_gb_honda_elesys(1.0, 10.0)
    self.assertEqual(brake, 0.0)
    self.assertAlmostEqual(gas, 1.0 / 4.8, places=6)

  def test_mutually_exclusive_and_clipped(self):
    for accel in np.arange(-4.0, 2.5, 0.25):
      for speed in (0.0, 0.5, 1.0, 2.0, 5.0, 15.0):
        gas, brake = compute_gb_honda_elesys(float(accel), speed)
        self.assertGreaterEqual(min(gas, brake), 0.0)
        self.assertLessEqual(max(gas, brake), 1.0)
        self.assertEqual(min(gas, brake), 0.0)  # never both

  def test_stopping_ramp_saturates(self):
    # stopAccel (-2.0) near standstill must be full brake (creep + 2.0 > 2.6)
    _, brake = compute_gb_honda_elesys(-2.0, 0.4)
    self.assertEqual(brake, 1.0)

  def test_light_gas_still_brakes_in_creep_band(self):
    # wanting +0.3 m/s^2 at crawl speed means braking: the car creeps harder than that on its own
    gas, brake = compute_gb_honda_elesys(0.3, 0.5)
    self.assertEqual(gas, 0.0)
    self.assertGreater(brake, 0.0)

  def test_launch_crossover_fades_creep(self):
    # at standstill the brake->gas crossover must sit near 0.5 m/s^2 of demand, not the full
    # creep offset (1.15) -- holding the brakes until a_cmd > creep caused ~1.2 s launch lag
    gas, brake = compute_gb_honda_elesys(0.55, 0.0)
    self.assertEqual(brake, 0.0)
    self.assertGreater(gas, 0.0)
    # below the crossover we still hold some brake (hill/creep hold)
    gas, brake = compute_gb_honda_elesys(0.2, 0.0)
    self.assertEqual(gas, 0.0)
    self.assertGreater(brake, 0.0)

  def test_strong_demand_has_no_creep_subtraction(self):
    # once demand exceeds the fade band, gas must be the plain scale (no double-counting creep)
    gas, brake = compute_gb_honda_elesys(1.6, 1.5)
    self.assertEqual(brake, 0.0)
    self.assertAlmostEqual(gas, 1.6 / GOLD_GAS_SCALE, places=6)

  def test_braking_path_matches_golden_table(self):
    # the fade must not alter any braking behavior (accel <= 0): pure golden-table offset
    for accel in np.arange(-3.0, 0.001, 0.125):
      for speed in np.arange(0.0, 6.0, 0.25):
        creep = float(np.interp(speed, GOLD_CREEP_BP, GOLD_CREEP_V))
        expect = float(np.clip(-(accel - creep) / GOLD_BRAKE_SCALE, 0.0, 1.0))
        _, brake = compute_gb_honda_elesys(float(accel), float(speed))
        self.assertAlmostEqual(brake, expect, places=9)

  def test_net_monotonic_no_overlap(self):
    # net demand must be monotonic in accel (single crossover, gas and brake can't fight)
    for speed in (0.0, 0.5, 1.0, 2.0, 4.0):
      last_net = -1e9
      for accel in np.arange(-2.0, 2.01, 0.05):
        gas, brake = compute_gb_honda_elesys(float(accel), speed)
        net = gas * GOLD_GAS_SCALE - brake * GOLD_BRAKE_SCALE
        self.assertGreaterEqual(net, last_net - 1e-9)
        last_net = net

  def test_deployed_creep_matches_golden_table(self):
    # probe the deployed table through the function: at accel=0 the fade factor is 1, so
    # brake * GOLD_BRAKE_SCALE == creep(v). Pins the inline values against the measurement.
    for v in list(GOLD_CREEP_BP) + [0.375, 1.25, 2.375, 4.0, 6.0, 10.0]:
      expect = float(np.interp(v, GOLD_CREEP_BP, GOLD_CREEP_V))
      _, brake = compute_gb_honda_elesys(0.0, float(v))
      self.assertAlmostEqual(brake * GOLD_BRAKE_SCALE, expect, places=6, msg=f'v={v}')


class TestBrakePumpHysteresis(unittest.TestCase):
  """Upstream pump logic must stay pristine; the Elesys variant trades the 20 s bleed window
  for short runs (0.5 s) at a load-scaled period with a jitter deadband.

  Two independent mechanisms, and they must not be confused:

  1. The CONTINUOUS-RUN branches (v >= 2.5 and cb > 200, or 0.15 <= v < 2.5 and cb > 100) pin
     the pump on at firm braking. A v4 experiment deleted them on the grounds that they were
     rare; that silently reverted commit 2905e73d1 and cost duty exactly where demand is
     highest. Measured v3 -> v4 on the real 13-route stream: at cb >= 200 duty 1.00 -> 0.32 and
     worst pump-off gap 0.16 s -> 5.50 s. RESTORED.

  2. The GRADED DEADBAND ([12, 6, 3] over cb [0, 60, 200]) governs LIGHT braking, where the
     audible whine actually lives -- `rise and in_run` re-primes inside the current run, so a
     slowly-drifting command used to pin the pump on. Cuts light-braking run time 15.75 ->
     12.10 min with no change to the worst gap.

  The two are orthogonal, which is why both are in. Total pump run is still 38.1 min against
  v3's 58.9 on the same stream."""

  def _duty_elesys(self, cb, seconds=20.0, jitter=0, v_ego=5.0):
    anchor, last_pump_ts, on = 0, -99.0, 0
    n = int(seconds * 100)
    for i in range(n):
      c = cb + (jitter if (i // 25) % 2 else -jitter)   # slow +-jitter square wave
      pump, anchor, last_pump_ts = brake_pump_hysteresis_elesys(c, v_ego, anchor, last_pump_ts, i * 0.01)
      on += pump
    return on / n

  def test_steady_duty_backstop_only(self):
    # perfectly steady command: only the periodic backstop runs (0.5 s per 12->6 s period)
    self.assertAlmostEqual(self._duty_elesys(60), 0.5 / 11.25, delta=0.02)
    self.assertAlmostEqual(self._duty_elesys(200), 0.5 / 6.0, delta=0.02)

  def test_steady_duty_is_lower_than_v3(self):
    # the whole point of v4: steady-state duty must be well under the v3 values it replaces
    # (v3 was 1.0 s per 8->5 s == 0.13 light / 0.20 firm)
    self.assertLess(self._duty_elesys(60), 0.09)
    self.assertLess(self._duty_elesys(200), 0.12)

  def test_jitter_does_not_retrigger(self):
    # +-2 count jitter must not add duty beyond the periodic refresh
    self.assertAlmostEqual(self._duty_elesys(100, jitter=2), self._duty_elesys(100, jitter=0), delta=0.03)

  def test_rise_triggers_and_holds(self):
    # a genuine ramp (+3/frame) keeps the pump on continuously
    anchor, last = 0, -99.0
    pumps = []
    for i in range(1, 60):
      p, anchor, last = brake_pump_hysteresis_elesys(3 * i, 5.0, anchor, last, i * 0.01)
      pumps.append(p)
    self.assertTrue(all(pumps[1:]))

  def test_refractory_blocks_small_rises_then_allows(self):
    # small rise right after a run: blocked by the 3.0 s refractory; allowed once idle
    # (run 0.5 + refractory 3.0 == the same 3.5 s idle threshold as v3),
    # and a big rise (+15, real apply ramp) bypasses the refractory entirely.
    # NB the rise used here is +10 at cb 50, because v5 scales the deadband with command
    # level and +6 no longer clears it down there -- see test_deadband_scales_with_command.
    anchor, last = 0, -99.0
    _, anchor, last = brake_pump_hysteresis_elesys(150, 5.0, anchor, last, 0.0)
    _, anchor, last = brake_pump_hysteresis_elesys(50, 5.0, anchor, last, 1.5)    # release re-arms anchor
    p, anchor, last = brake_pump_hysteresis_elesys(60, 5.0, anchor, last, 1.6)    # +10: refractory blocks
    self.assertFalse(p)
    p, anchor, last = brake_pump_hysteresis_elesys(60, 5.0, anchor, last, 4.0)    # idle: now allowed
    self.assertTrue(p)

  def test_deadband_scales_with_command(self):
    # v5. `rise and in_run` re-primes inside the current run, so a command creeping up by the
    # deadband every 0.5 s pins the pump on. With a flat deadband of 3 that made light braking
    # 48.2% of all moving run time (405.9 min engaged, routes 15646e8515eda1a7). Requiring a
    # bigger rise at low command breaks that, and leaves firm braking exactly as it was.
    def rises(anchor_cb, delta):
      """does a rise of `delta` from `anchor_cb` re-prime once idle?"""
      anchor, last = 0, -99.0
      _, anchor, last = brake_pump_hysteresis_elesys(anchor_cb, 5.0, anchor, last, 0.0)
      p, _, _ = brake_pump_hysteresis_elesys(anchor_cb + delta, 5.0, anchor, last, 10.0)
      return p

    # light braking: small drift must NOT re-prime, a real rise still must
    self.assertFalse(rises(30, 5))
    self.assertFalse(rises(30, 8))
    self.assertTrue(rises(30, 14))
    # firm braking: unchanged from v4, +4 is still enough
    self.assertTrue(rises(210, 4))
    self.assertTrue(rises(160, 6))
    # and the deadband is monotonically decreasing in command level
    from opendbc.car.honda.carcontroller import ELESYS_PUMP_DEADBAND_BP, ELESYS_PUMP_DEADBAND_V
    self.assertEqual(ELESYS_PUMP_DEADBAND_V, sorted(ELESYS_PUMP_DEADBAND_V, reverse=True))
    self.assertEqual(len(ELESYS_PUMP_DEADBAND_BP), len(ELESYS_PUMP_DEADBAND_V))

  def test_apply_ramp_into_a_stop_still_pins_the_pump(self):
    # the whole risk of a bigger low-command deadband is delaying pressure build. A genuine
    # apply ramp climbs far faster than the deadband, so it must still run continuously.
    anchor, last, on = 0, -99.0, 0
    for i in range(150):
      p, anchor, last = brake_pump_hysteresis_elesys(40 + i, 2.2 - i * 0.012, anchor, last, i * 0.01)
      on += p
    self.assertGreater(on / 150, 0.95)
    anchor, last = 0, -99.0
    _, anchor, last = brake_pump_hysteresis_elesys(150, 5.0, anchor, last, 0.0)
    p, anchor, last = brake_pump_hysteresis_elesys(170, 5.0, anchor, last, 1.6)   # +20: bypasses refractory
    self.assertTrue(p)

  def test_final_approach_at_firm_command_is_continuous(self):
    # REGRESSION for the v4 experiment that deleted this branch. Firm braking through the stop
    # approach (0.15 <= v < 2.5, cb > 100) must pin the pump on -- one smooth run that fades into
    # the hold. v4 let the ordinary rise/backstop rules govern instead, on the argument that the
    # branch was rare; but the frames it covers are the maximum-demand ones, and once the command
    # is firm the rise-trigger is dead. Measured v3 -> v4 on the real routes: stop-approach duty
    # 0.72 -> 0.26 and worst pump-off gap 7.00 s -> 10.22 s. Restored.
    anchor, last, starts, on, prev = 0, -99.0, 0, 0, False
    for i in range(500):  # 5 s at v ramping 2.4 -> 0.2
      v = 2.4 - i * 0.0044
      p, anchor, last = brake_pump_hysteresis_elesys(150, v, anchor, last, i * 0.01)
      starts += (p and not prev)
      on += p
      prev = p
    self.assertGreater(on / 500, 0.95)
    self.assertEqual(starts, 1)

  def test_light_command_in_the_approach_is_not_continuous(self):
    # the branch is deliberately gated on cb > 100, so a LIGHT command at walking pace -- the
    # audible case that motivated the v4 experiment -- still falls through to the ordinary rules
    anchor, last, on = 0, -99.0, 0
    for i in range(500):
      v = 2.4 - i * 0.0044
      p, anchor, last = brake_pump_hysteresis_elesys(60, v, anchor, last, i * 0.01)
      on += p
    self.assertLess(on / 500, 0.35)

  def test_final_approach_still_primes_on_a_real_ramp(self):
    # the saving must not come out of pressure build: a genuine apply ramp into a stop still
    # keeps the pump running, because each +3 rise re-primes inside the current run
    anchor, last, on = 0, -99.0, 0
    for i in range(150):  # 1.5 s ramping cb 60 -> 210 while slowing through the band
      p, anchor, last = brake_pump_hysteresis_elesys(60 + i, 2.2 - i * 0.012, anchor, last, i * 0.01)
      on += p
    self.assertGreater(on / 150, 0.95)

  def test_standstill_hold_is_quiet(self):
    # stopped with brake held: rare top-ups only (~1 s per 30 s), not a burp every 2.5 s.
    # 487 s of logged holds showed zero effective creep across every pump-off gap.
    duty = self._duty_elesys(250, seconds=60.0, v_ego=0.0)
    self.assertLess(duty, 0.06)

  def test_motion_reprimes_after_long_hold(self):
    # if pressure ever decays enough that the car starts creeping, the standstill period no
    # longer applies and the expired timer must re-prime immediately
    anchor, last = 0, -99.0
    _, anchor, last = brake_pump_hysteresis_elesys(250, 0.0, anchor, last, 0.0)    # prime on entry
    p, anchor, last = brake_pump_hysteresis_elesys(250, 0.0, anchor, last, 10.0)   # deep in the hold: quiet
    self.assertFalse(p)
    p, anchor, last = brake_pump_hysteresis_elesys(250, 0.3, anchor, last, 10.1)   # car creeps -> re-prime
    self.assertTrue(p)

  def test_saturated_moving_braking_pumps_continuously(self):
    # REGRESSION for commit 2905e73d1 "Fixed Pump Blind Spot on Saturated Braking", which the v4
    # experiment silently reverted. At a railed command the rise-trigger has nothing left to rise
    # to, so without this branch only the 6 s backstop runs. Measured v3 -> v4 on the real
    # 13-route command stream at cb >= 200: duty 1.00 -> 0.32, worst pump-off gap 0.16 s -> 5.50 s.
    # That is the exact ~3 s-gap signature that bled 0.5-0.7 m/s^2 and caused the end-of-stop bite.
    duty = self._duty_elesys(253, seconds=30.0, v_ego=15.0)
    self.assertGreater(duty, 0.95)

  def test_saturation_threshold_boundary(self):
    # the branch is cb > 200 exactly: at 200 the ordinary rules still govern, so the deletion
    # cannot be reintroduced by accident via an off-by-one
    self.assertGreater(self._duty_elesys(201, seconds=30.0, v_ego=15.0), 0.95)
    self.assertLess(self._duty_elesys(200, seconds=30.0, v_ego=15.0), 0.5)

  def test_backstop_is_load_scaled_not_flat(self):
    # firm braking must refresh sooner than light braking. This is not cosmetic: the backstop is
    # what re-primes a hold the instant the car creeps (see test_motion_reprimes_after_long_hold),
    # so a flat period would lengthen that recovery for a firm hold.
    self.assertGreater(self._duty_elesys(253, v_ego=15.0), self._duty_elesys(60, v_ego=15.0))

  def test_saturated_standstill_stays_quiet(self):
    # firm command at v=0 must still keep the 30 s top-up period, not the moving backstop
    self.assertLess(self._duty_elesys(253, seconds=60.0, v_ego=0.0), 0.04)

  def test_release_from_firm_braking_stops_pumping(self):
    # leaving firm braking: the current run tails out (<= 0.5 s), then the refractory and
    # backstop govern again -- the pump must not stay latched on
    anchor, last = 0, -99.0
    for i in range(200):  # 2 s firm at speed
      _, anchor, last = brake_pump_hysteresis_elesys(253, 15.0, anchor, last, i * 0.01)
    p, anchor, last = brake_pump_hysteresis_elesys(60, 15.0, anchor, last, 3.2)  # released, 1.2 s later
    self.assertFalse(p)

  def test_run_length_is_half_of_v3(self):
    # pin the headline constant: a single isolated prime runs 0.5 s, not 1.0 s
    anchor, last = 0, -99.0
    _, anchor, last = brake_pump_hysteresis_elesys(150, 15.0, anchor, last, 0.0)
    p, _, _ = brake_pump_hysteresis_elesys(150, 15.0, anchor, last, 0.45)
    self.assertTrue(p)
    p, _, _ = brake_pump_hysteresis_elesys(150, 15.0, anchor, last, 0.55)
    self.assertFalse(p)

  def test_no_pump_without_brake(self):
    p, _, _ = brake_pump_hysteresis_elesys(0, 0.0, 0, -99.0, 5.0)
    self.assertFalse(p)
    p2, _ = brake_pump_hysteresis(0, 0, 0.0, 5.0)
    self.assertFalse(p2)

  def test_upstream_default_unchanged(self):
    # upstream: steady state barely pumps (0.2 s per 20 s), rising always pumps
    last, on = 0.0, 0
    for i in range(2000):
      pump, last = brake_pump_hysteresis(100, 100, last, i * 0.01)
      on += pump
    self.assertLess(on / 2000, 0.06)
    last = 0.0
    for i in range(1, 100):
      pump, last = brake_pump_hysteresis(i + 1, i, last, i * 0.01)
      self.assertTrue(pump)

class TestElesysGasMultiplier(unittest.TestCase):
  """Golden pedal-multiplier curve -- deliberately duplicated so any change to the deployed
  curve fails a test until it has been re-measured.

  Re-measured 2026-08 against the full 14-route set. Settled-frame plant identification
  (target steady >= 1.5 s, demand = aEgo + g*sin(pitch) + aero(v) regressed on the interceptor
  command, ~200k frames over three routes on two tunes) put the command actually needed at
  1.38x @ 3-6 m/s, 1.30-1.47x @ 6-10, 1.59-1.70x @ 10-15, 1.51x @ 15-20 and 1.50-1.75x @ 20+,
  with actuatorsOutput.gas never reaching 0.9 on any of the 13 engaged routes -- so nothing
  physical was capping it. The top three breakpoints were raised ~1.25x, one measured step
  rather than the full ratio, because the PI and the pitch feedforward close part of the rest.
  Previous golden was [0.55, 0.85, 1.10, 1.25, 1.55, 2.20]."""
  GOLD_MULT_BP = [0., 3., 6., 10., 15., 20.]
  GOLD_MULT_V = [0.55, 0.85, 1.20, 1.55, 1.95, 2.75]

  def test_deployed_curve_matches_golden(self):
    for v in list(self.GOLD_MULT_BP) + [1.5, 4.5, 8.0, 12.5, 17.5, 25.0]:
      expect = float(np.interp(v, self.GOLD_MULT_BP, self.GOLD_MULT_V))
      self.assertAlmostEqual(elesys_gas_multiplier(float(v)), expect, places=6, msg=f'v={v}')

  def test_monotonic_increasing(self):
    # gain falloff with speed means the multiplier must only ever grow
    vals = [elesys_gas_multiplier(float(v)) for v in np.arange(0.0, 25.0, 0.25)]
    for a, b in zip(vals[:-1], vals[1:], strict=True):
      self.assertGreaterEqual(b, a - 1e-9)

  def test_mid_band_above_old_curve(self):
    # the 3-14 m/s band delivered only ~50-65% of commanded accel on the old <=1.0 ramp;
    # the fix is specifically that this band now multiplies above it
    self.assertGreaterEqual(elesys_gas_multiplier(0.0), 0.5)
    for v in (6.0, 8.0, 10.0, 12.0):
      old = float(np.interp(v, [0., 10., 15., 20.], [0.5, 1.0, 1.4, 2.1]))
      self.assertGreater(elesys_gas_multiplier(v), old, msg=f'v={v}')


class TestBrakeCommandUnitsBit(unittest.TestCase):
  """SET_ME_1 in BRAKE_COMMAND is the cluster units flag on Elesys cars (0 = metric,
  1 = imperial) and a reserved constant 1 on every other Honda."""

  def _frame(self, car, is_metric):
    # fresh packer per frame so counters match and frames are byte-comparable
    try:
      from opendbc.can import CANPacker
      packer = CANPacker(DBC[car][Bus.pt])
    except Exception as e:  # compiled packer not available in this environment
      self.skipTest(f'opendbc.can unavailable: {e}')
    CAN = SimpleNamespace(pt=0)
    CP_SP = SimpleNamespace(flags=0)
    msg = hondacan.create_brake_command(packer, CAN, 100, True, True, False, 0, car, {}, is_metric, CP_SP)
    return bytes(msg.dat if hasattr(msg, 'dat') else msg[1])

  def _units_bit_mask(self, car):
    # locate the SET_ME_1 bit by packing it 0 vs 1 with everything else zero
    from opendbc.can import CANPacker
    a = CANPacker(DBC[car][Bus.pt]).make_can_msg("BRAKE_COMMAND", 0, {"SET_ME_1": 0})
    b = CANPacker(DBC[car][Bus.pt]).make_can_msg("BRAKE_COMMAND", 0, {"SET_ME_1": 1})
    da = bytes(a.dat if hasattr(a, 'dat') else a[1])
    db = bytes(b.dat if hasattr(b, 'dat') else b[1])
    return bytes(x ^ y for x, y in zip(da, db, strict=True))

  def test_elesys_bit_tracks_units(self):
    metric = self._frame(CAR.HONDA_ACCORD_9G_AU, True)
    imperial = self._frame(CAR.HONDA_ACCORD_9G_AU, False)
    mask = self._units_bit_mask(CAR.HONDA_ACCORD_9G_AU)
    diff = bytes(x ^ y for x, y in zip(metric, imperial, strict=True))
    # outside the checksum byte, metric vs imperial frames differ in exactly the SET_ME_1 bit
    self.assertEqual(diff[:7], mask[:7])
    self.assertEqual(sum(bin(b).count('1') for b in mask[:7]), 1)
    # metric = bit clear (matches the stock radar on the metric AU car), imperial = bit set
    idx = next(i for i, b in enumerate(mask[:7]) if b)
    self.assertEqual(metric[idx] & mask[idx], 0)
    self.assertEqual(imperial[idx] & mask[idx], mask[idx])

  def test_other_hondas_unaffected(self):
    # non-Elesys cars: constant 1 regardless of units, so frames are identical
    self.assertEqual(self._frame(CAR.HONDA_CRV, True), self._frame(CAR.HONDA_CRV, False))

class TestElesysGearDecode(unittest.TestCase):
  """GEARBOX_AUTO raw 0 means BOTH Sport and between-detents on this car, so only
  time separates them. Across 473k logged frames every 0 run was a shift transient:
  47 of them, median 3 frames (30 ms), max 520 ms, all below 0.5 m/s -- S was never
  selected during the recording."""

  @staticmethod
  def _name(gear):
    """capnp enums are bare ints once assigned; map back through the schema."""
    from opendbc.car import structs
    names = {int(v): k for k, v in structs.CarState.GearShifter.schema.enumerants.items()}
    raw = getattr(gear, "raw", None)
    return names.get(int(raw if raw is not None else gear))

  def _cs(self):
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.honda.carstate import CarState
    from opendbc.car.honda.interface import CarInterface
    fp = gen_empty_fingerprint()
    fp[0][0x188] = 8
    CP = CarInterface.get_params(CAR.HONDA_ACCORD_9G_AU, fp, [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.HONDA_ACCORD_9G_AU, fp, [], False, False, False)
    self.assertEqual(str(CP.transmissionType), 'automatic')
    return CarState(CP, CP_SP)

  def test_detents_decode(self):
    cs = self._cs()
    for raw, name in ((1, 'park'), (2, 'reverse'), (4, 'neutral'), (8, 'drive')):
      self.assertEqual(self._name(cs.update_gear_elesys(raw, 0)), name, msg=f'raw={raw}')

  def test_shift_transient_holds_last_gear(self):
    # the real worst case: 52 frames (520 ms) of raw 0 between two detents
    cs = self._cs()
    cs.update_gear_elesys(8, 4)                      # in D
    for _ in range(52):
      self.assertEqual(self._name(cs.update_gear_elesys(0, 0)), 'drive')
    self.assertEqual(self._name(cs.update_gear_elesys(4, 3)), 'neutral')

  def test_sustained_zero_becomes_sport(self):
    cs = self._cs()
    cs.update_gear_elesys(8, 4)
    for _ in range(cs.SPORT_DWELL - 1):
      self.assertEqual(self._name(cs.update_gear_elesys(0, 0)), 'drive')
    self.assertEqual(self._name(cs.update_gear_elesys(0, 0)), 'sport')   # dwell reached

  def test_gear_26_is_an_instant_fast_path(self):
    cs = self._cs()
    cs.update_gear_elesys(8, 4)
    self.assertEqual(self._name(cs.update_gear_elesys(0, 26)), 'sport')  # no dwell needed

  def test_leaving_sport_is_immediate_and_rearms(self):
    cs = self._cs()
    cs.update_gear_elesys(8, 4)
    for _ in range(cs.SPORT_DWELL):
      cs.update_gear_elesys(0, 0)
    self.assertEqual(self._name(cs.gear_shifter_last), 'sport')
    self.assertEqual(self._name(cs.update_gear_elesys(8, 4)), 'drive')   # back to D at once
    self.assertEqual(cs.gear_zero_frames, 0)                      # counter re-armed
    for _ in range(10):                                           # a short blip stays D
      self.assertEqual(self._name(cs.update_gear_elesys(0, 0)), 'drive')

  def test_dwell_is_clear_of_the_longest_observed_transient(self):
    cs = self._cs()
    self.assertGreater(cs.SPORT_DWELL * 0.01, 0.52 * 1.5)


class TestElesysStockAeb(unittest.TestCase):
  """stockAeb stands openpilot down so the factory CMBS can have the car. The four bits below
  are the ones confirmed on this car by bit-level analysis of a real event; all four read 0
  across 351,809 stock BRAKE_COMMAND frames on bus 2, so none of them can fire spuriously."""

  BITS = ("CMBS_BRAKE", "AEB_REQ_3", "AEB_REQ_2", "AEB_STATUS")

  @staticmethod
  def _stock_aeb(**kw):
    """mirrors the expression in carstate.py"""
    b = {"CMBS_BRAKE": 0, "AEB_REQ_3": 0, "AEB_REQ_2": 0, "AEB_STATUS": 0,
         "FCW": 0, "COMPUTER_BRAKE": 0}
    b.update(kw)
    return bool(b["CMBS_BRAKE"] or b["AEB_REQ_3"] or b["AEB_REQ_2"] or b["AEB_STATUS"] == 1)

  def test_quiet_when_nothing_is_asserted(self):
    self.assertFalse(self._stock_aeb())

  def test_each_confirmed_bit_triggers_on_its_own(self):
    self.assertTrue(self._stock_aeb(CMBS_BRAKE=1))
    self.assertTrue(self._stock_aeb(AEB_REQ_3=1))
    self.assertTrue(self._stock_aeb(AEB_REQ_2=1))
    self.assertTrue(self._stock_aeb(AEB_STATUS=1))

  def test_does_not_wait_for_computer_brake(self):
    # the whole point of the request bits: catch the event before pressure rises
    self.assertTrue(self._stock_aeb(AEB_REQ_3=1, COMPUTER_BRAKE=0))

  def test_warnings_alone_do_not_stand_openpilot_down(self):
    # measured: every non-zero AEB_STATUS / FCW frame in 351,809 stock frames was
    # AEB_STATUS=3 (aeb_prepare) and/or FCW=2 with COMPUTER_BRAKE=0 -- a warning, not braking
    self.assertFalse(self._stock_aeb(AEB_STATUS=3, FCW=2, COMPUTER_BRAKE=0))
    self.assertFalse(self._stock_aeb(AEB_STATUS=2))
    self.assertFalse(self._stock_aeb(FCW=2))

  def test_aeb_req_1_is_not_used(self):
    # bit 29 is the generic Nidec position and never asserts on this car; bit 27 is the real one
    self.assertFalse(self._stock_aeb(AEB_REQ_1=1))

  def test_all_four_bits_are_defined_in_the_dbc(self):
    from opendbc.can import CANParser
    p = CANParser(DBC[ELESYS_CAR][Bus.pt], [], 0)
    sigs = p.vl["BRAKE_COMMAND"]
    for s in self.BITS:
      self.assertIn(s, sigs, msg=f"{s} missing from BRAKE_COMMAND")


if __name__ == "__main__":
  unittest.main()
