import unittest

import numpy as np

from types import SimpleNamespace

from opendbc.car import Bus
from opendbc.car.honda import hondacan
from opendbc.car.honda.carcontroller import (ELESYS_BRAKE_SCALE, ELESYS_CREEP_BP, ELESYS_CREEP_V,
                                             brake_pump_hysteresis, compute_gas_brake,
                                             compute_gb_honda_elesys, compute_gb_honda_nidec)
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
    self.assertAlmostEqual(brake, 1.5 / ELESYS_BRAKE_SCALE, places=6)

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

  def test_creep_table_shape(self):
    self.assertEqual(len(ELESYS_CREEP_BP), len(ELESYS_CREEP_V))
    self.assertEqual(ELESYS_CREEP_V[-1], 0.0)          # no offset at speed
    self.assertTrue(all(np.diff(ELESYS_CREEP_BP) > 0))  # monotonic BP
    self.assertTrue(all(np.diff(ELESYS_CREEP_V) <= 0))  # creep only fades with speed


class TestBrakePumpHysteresis(unittest.TestCase):
  def _duty(self, steady_refresh, seconds=5.0):
    last_pump_ts, on = 0.0, 0
    n = int(seconds * 100)
    for i in range(n):
      pump, last_pump_ts = brake_pump_hysteresis(100, 100, last_pump_ts, i * 0.01, steady_refresh)
      on += pump
    return on / n

  def test_steady_duty_elesys_matches_stock(self):
    # stock Elesys runs ~39% pump duty during steady braking; 0.5 s refresh gives ~40%
    self.assertAlmostEqual(self._duty(0.5), 0.40, delta=0.03)

  def test_steady_duty_upstream_default(self):
    # upstream 20 s refresh: pump barely runs at steady state (the bleed window)
    self.assertLess(self._duty(20.0), 0.06)

  def test_rising_always_pumps(self):
    last_pump_ts = 0.0
    for i in range(1, 100):
      pump, last_pump_ts = brake_pump_hysteresis(i + 1, i, last_pump_ts, i * 0.01, 20.0)
      self.assertTrue(pump)

  def test_no_pump_without_brake(self):
    pump, _ = brake_pump_hysteresis(0, 0, 0.0, 5.0, 0.5)
    self.assertFalse(pump)


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
