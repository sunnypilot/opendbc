import unittest

import numpy as np

from types import SimpleNamespace

from opendbc.car import Bus
from opendbc.car.honda import hondacan
from opendbc.car.honda.carcontroller import (brake_pump_hysteresis, brake_pump_hysteresis_elesys,
                                             compute_gas_brake, compute_gb_honda_elesys,
                                             compute_gb_honda_nidec)

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
  for stock-like long runs (1.0 s) at a load-scaled period with a jitter deadband."""

  def _duty_elesys(self, cb, seconds=20.0, jitter=0):
    anchor, last_pump_ts, on = 0, -99.0, 0
    n = int(seconds * 100)
    for i in range(n):
      c = cb + (jitter if (i // 25) % 2 else -jitter)   # slow +-jitter square wave
      pump, anchor, last_pump_ts = brake_pump_hysteresis_elesys(c, anchor, last_pump_ts, i * 0.01)
      on += pump
    return on / n

  def test_steady_duty_load_scaled(self):
    # 1.0 s runs: light braking ~1/4.0 s, firm ~1/2.5 s (stock's duty gradient direction)
    self.assertAlmostEqual(self._duty_elesys(60), 1.0 / 3.81, delta=0.05)
    self.assertAlmostEqual(self._duty_elesys(200), 1.0 / 2.5, delta=0.05)

  def test_jitter_does_not_retrigger(self):
    # +-2 count jitter must not add duty beyond the periodic refresh
    self.assertAlmostEqual(self._duty_elesys(100, jitter=2), self._duty_elesys(100, jitter=0), delta=0.03)

  def test_rise_triggers_and_holds(self):
    # a genuine ramp (+3/frame) keeps the pump on continuously
    anchor, last = 0, -99.0
    pumps = []
    for i in range(1, 60):
      p, anchor, last = brake_pump_hysteresis_elesys(3 * i, anchor, last, i * 0.01)
      pumps.append(p)
    self.assertTrue(all(pumps[1:]))

  def test_release_rearms_trigger(self):
    # command drops then rises again: rise must re-trigger promptly (anchor follows down)
    anchor, last = 0, -99.0
    _, anchor, last = brake_pump_hysteresis_elesys(150, anchor, last, 0.0)
    _, anchor, last = brake_pump_hysteresis_elesys(50, anchor, last, 1.5)   # released past run window
    p, anchor, last = brake_pump_hysteresis_elesys(56, anchor, last, 1.6)   # +6 over new anchor
    self.assertTrue(p)

  def test_no_pump_without_brake(self):
    p, _, _ = brake_pump_hysteresis_elesys(0, 0, -99.0, 5.0)
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


if __name__ == "__main__":
  unittest.main()
