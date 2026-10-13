"""
Behavioural tests for Toyota longitudinal control, driven through the real CarController.

These are the safety net that makes tuning observable: every accelerator request in the
schedule is checked at every speed, so a change to the PID tune, the rate limits or the
brake onset shows up here instead of on the car.

They assert behaviour, not constants - the numbers are the loop's output, measured
through the carcontroller the way the car produces it.
"""

import unittest
from unittest import mock

import numpy as np
from opendbc.car import DT_CTRL
from opendbc.car.toyota.carcontroller import ACCEL_PID_UNWIND, ACCEL_WINDDOWN_LIMIT
from opendbc.sunnypilot.car.toyota.tests import long_control_plant as plant
from opendbc.sunnypilot.car.toyota.tests.long_control_harness import LONG_DT, LongHarness

# the whole schedule: creep-adjacent, urban, rural, highway
SPEEDS_KMH = (20, 40, 60, 80, 108)

# the stock down jerk, for comparing against the softened onset
STOCK_DOWN_JERK = -ACCEL_WINDDOWN_LIMIT / LONG_DT


class LongControlTestCase(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.h = LongHarness()
    # the same carcontroller with the brake-onset module gated off, i.e. stock. The
    # plant tests measure against this so a behaviour is pinned to the car, not to the
    # corrections; TestRestoredCorrections uses the pair to measure each correction.
    cls.stock = LongHarness(brake_onset=False)

  @staticmethod
  def _jerk_budget(v_kmh):
    """Largest onset jerk allowed.

    The shaper ramps up to the stock limit, so the achieved onset jerk tracks the stock
    value rather than the schedule. These bounds sit just under what the unshaped path
    produces (3.2 m/s^3 at every speed), so the test fails if the onset stops softening.
    """
    return 2.2 if v_kmh <= 60 else 3.05

  def steps(self, a_to, seconds=10.0):
    for v_kmh in SPEEDS_KMH:
      with self.subTest(request=a_to, v_kmh=v_kmh):
        yield v_kmh, self.h.step(0.0, a_to, v_kmh / 3.6, seconds=seconds)


class TestTracking(LongControlTestCase):
  def test_requested_accel_is_delivered_at_every_speed(self):
    """The car must not systematically over-brake or under-gas.

    Measured as requested-minus-achieved accel, so it is not hidden by the integral
    legitimately trimming the disturbance. Only light requests are checked: the plant
    models the car's regen adding decel in proportion to how hard it is braking, which
    no amount of integrator can trim (any command produces proportionally more) - that
    belongs to TestPlantSeesTheCarBehaviours and is what a feed-forward limiter is for.
    """
    for a_to in (-1.0, 1.0):
      for v_kmh, tr in self.steps(a_to):
        with self.subTest(request=a_to, v_kmh=v_kmh):
          delivered = tr.steady_state(tr.t[-1] - 1.0)[1]
          self.assertLess(abs(tr.tracking_error(tr.t[-1] - 1.0)), 0.06,
                          f"{a_to:+.1f} m/s^2 at {v_kmh} km/h is delivered as {delivered:+.3f}")

  def test_a_large_brake_request_is_actually_reached(self):
    """The soft onset may delay a brake, but the decel must still arrive."""
    for v_kmh, tr in self.steps(-2.0):
      with self.subTest(v_kmh=v_kmh):
        self.assertLess(tr.peak_accel(), -1.9, f"a -2.0 request only reached {tr.peak_accel():.3f} at {v_kmh} km/h")


class TestBrakeOnset(LongControlTestCase):
  """The onset must be gentle, and an emergency must not be."""

  def test_onset_jerk_is_softened(self):
    for v_kmh, tr in self.steps(-2.0):
      with self.subTest(v_kmh=v_kmh):
        jerk = tr.onset_jerk()
        self.assertLess(jerk, self._jerk_budget(v_kmh),
                        f"brake bites at {jerk:.2f} m/s^3 at {v_kmh} km/h (unshaped {STOCK_DOWN_JERK:.2f})")

  def test_soft_onset_costs_no_tracking(self):
    """Softening the onset must not make the steady state worse."""
    for v_kmh in SPEEDS_KMH:
      with self.subTest(v_kmh=v_kmh):
        tr = self.h.step(0.0, -1.0, v_kmh / 3.6)
        self.assertLess(tr.overshoot(), 0.07, f"overshoot {tr.overshoot():.3f} at {v_kmh} km/h")
        self.assertLess(abs(tr.tracking_error(tr.t[-1] - 1.0)), 0.06)


class TestEmergencyBraking(LongControlTestCase):
  """A request past HARD_BRAKE_ACCEL must leave the soft onset behind.

  This is the property that keeps the soft onset from being a safety regression. The
  emergency path still passes through URGENT_J for URGENT_T, so it is not bit-identical
  to stock - the test bounds how far it may differ and how late it may be.
  """

  def test_a_hard_request_is_never_late_or_shallow(self):
    """An emergency must reach full braking about as fast as stock does."""
    for v_kmh in SPEEDS_KMH:
      with self.subTest(v_kmh=v_kmh):
        shaped = self.h.step(0.0, -3.5, v_kmh / 3.6, seconds=8.0)
        stock = self.stock.step(0.0, -3.5, v_kmh / 3.6, seconds=8.0)
        delay = shaped.time_to(-2.8) - stock.time_to(-2.8)
        self.assertLess(delay, 0.2, f"-2.8 m/s^2 arrives {delay:.2f}s later than stock at {v_kmh} km/h")
        self.assertGreater(shaped.peak_accel(), stock.peak_accel() - 0.3,
                           f"full braking is {stock.peak_accel() - shaped.peak_accel():.2f} m/s^2 shallower at {v_kmh} km/h")
        diff = float(np.abs(np.asarray(shaped.a_ego) - np.asarray(stock.a_ego)).max())
        self.assertLess(diff, 0.7, f"the emergency path differs from stock by {diff:.2f} m/s^2 at {v_kmh} km/h")

  def test_hard_request_reaches_full_braking_promptly(self):
    for v_kmh in SPEEDS_KMH:
      with self.subTest(v_kmh=v_kmh):
        t = self.h.step(0.0, -3.5, v_kmh / 3.6, seconds=8.0).time_to(-2.8)
        self.assertLess(t, 1.3, f"took {t:.2f}s to reach -2.8 m/s^2 at {v_kmh} km/h")

  def test_a_sub_emergency_request_is_still_softened(self):
    """-2.5 is a firm but not emergency brake, so it stays gentle."""
    tr = self.h.step(0.0, -2.5, 60 / 3.6)
    self.assertLess(tr.onset_jerk(), self._jerk_budget(60))


class TestResponseShape(LongControlTestCase):
  def test_brake_arrives_within_the_onset_budget(self):
    """The soft onset buys smoothness with time; the budget bounds that cost."""
    for v_kmh in SPEEDS_KMH:
      with self.subTest(v_kmh=v_kmh):
        tr = self.h.step(0.0, -1.0, v_kmh / 3.6)
        self.assertLess(tr.rise_time(), 2.0, f"1.0 m/s^2 step at {v_kmh} km/h took {tr.rise_time():.2f}s")

  def test_firm_brake_arrives_within_the_onset_budget(self):
    for v_kmh in SPEEDS_KMH:
      with self.subTest(v_kmh=v_kmh):
        tr = self.h.step(0.0, -2.0, v_kmh / 3.6)
        self.assertLess(tr.rise_time(), 3.0, f"2.0 m/s^2 step at {v_kmh} km/h took {tr.rise_time():.2f}s")


class TestSmoothness(LongControlTestCase):
  def test_steady_state_hold_has_no_limit_cycle(self):
    """A held request must produce a settled command, not a sustained cycle.

    The bound is the perceptible/DBC floor (0.02 m/s^2 peak to peak); the loop should be
    well under it. Note this does not cover oscillations that come from the planner or
    the lead - only the carcontroller's own loop.
    """
    for v_kmh in SPEEDS_KMH:
      with self.subTest(v_kmh=v_kmh):
        tr = self.h.hold(-1.0, v_kmh / 3.6, 12.0)
        ripple = tr.command_ripple(tr.t[-1] - 3.0)
        self.assertLess(ripple, 0.02, f"command ripples {ripple:.4f} m/s^2 at {v_kmh} km/h")


class TestWindup(LongControlTestCase):
  def test_recovers_after_a_sustained_maximum_brake(self):
    """A long maximum brake must not leave the loop stuck when it is released.

    The integrator leak exists for this; the reduction in the leak must not undo it.
    """
    for v_kmh in (20, 108):
      with self.subTest(v_kmh=v_kmh):
        tr = self.h.step(-3.5, 0.0, v_kmh / 3.6, warmup=20.0, seconds=12.0)
        achieved = tr.steady_state(tr.t[-1] - 1.0)[1]
        self.assertLess(tr.settle_time(band=0.15), 6.0,
                        f"still {achieved:+.3f} m/s^2 12s after releasing a maximum brake at {v_kmh} km/h")


class TestPlantSeesTheCarBehaviours(LongControlTestCase):
  """The plant must model what the carcontroller's corrections act on.

  Without these the harness reports "smooth" while being blind to the symptoms, so a
  restored correction would show no measurable change. Each test pins one behaviour the
  plant has to keep producing, against the request/delivered pairs in the logged stops.

  All of these run on the stock baseline: they are about the car, and they must hold
  whether or not the corrections are enabled - otherwise the correction would be hiding
  the very behaviour the test is asserting.
  """

  def test_regen_handover_dip_is_modelled(self):
    """Under-delivery through the low-speed band, back on target by 3 km/h.

    Logged stops show -0.84/-0.61, -0.87/-0.69, -0.75/-0.62, -0.60/-0.40 asked/delivered
    from 15 down to 4 km/h, and delivery back on the request by 3-2 km/h. The stock PID
    trims part of it, so the gap here is the delivered-minus-request one, not the raw
    shortfall the PCM produces.
    """
    tr = self.stock.stop(-0.8, 60 / 3.6)
    for lo, hi in ((15, 9), (9, 7), (7, 5)):
      with self.subTest(band=f"{lo}-{hi} km/h"):
        gap = tr.band_gap(lo, hi)
        self.assertGreater(gap, 0.05, f"no hand-over dip at {lo}-{hi} km/h (gap {gap:+.3f})")
    self.assertLess(tr.band_gap(3, 1), 0.05,
                    f"the hand-over dip must be gone by 3 km/h, got {tr.band_gap(3, 1):+.3f}")

  def test_firm_braking_over_delivery_is_modelled(self):
    """The car brakes harder than asked on a firm request, growing with the request.

    Logged: asked -1.46 -> -1.72, -2.26 -> -2.74, -3.49 -> -4.30.
    """
    over = {req: self.stock.stop(req, 80 / 3.6).over_delivery() for req in (-1.4, -2.2, -3.0)}
    self.assertLess(over[-1.4], 0.2, f"a moderate request is near the ask, got {over[-1.4]:+.3f}")
    self.assertGreater(over[-2.2], 0.3, f"firm braking should over-deliver, got {over[-2.2]:+.3f}")
    self.assertGreater(over[-3.0], 0.5, f"hard braking should over-deliver most, got {over[-3.0]:+.3f}")
    self.assertGreater(over[-2.2], over[-1.4], "over-delivery must grow with the request")
    self.assertGreater(over[-3.0], over[-2.2], "over-delivery must grow with the request")

  def test_friction_force_builds_on_entry(self):
    """The friction brake enters behind a fast command and the car's decel jolts with it."""
    tr = self.stock.stop(-1.0, 60 / 3.6)
    self.assertGreater(self.stock.plant.brake_force, 0.0, "the friction brake never engaged")
    # 27 logged entries: 3.70/2.22/1.22 m/s^3 of delivered jerk at 1.91/1.05/0.44 m/s^3 of
    # command rate, r = 0.78 - a stock command rate of ~4 m/s^3 gives ~8 m/s^3 of jolt.
    self.assertGreater(self.stock.plant.entry_jerk, 4.0,
                       f"the entry jolt is {self.stock.plant.entry_jerk:.2f} m/s^3")
    self.assertGreater(tr.delivery_wander(), 0.2,
                       f"the stop should wander relative to the request, got {tr.delivery_wander():.3f}")

  def test_a_compensation_would_be_visible(self):
    """Sensitivity: removing the hand-over must move the metric that measures it.

    This is what makes the harness usable for the corrections - if the dip could not be
    turned off, a restored correction could not be shown to fix anything.
    """
    seen = self.stock.stop(-0.8, 60 / 3.6).band_gap(15, 9)
    with mock.patch.object(plant, "HANDOVER_FRAC", [0.0] * len(plant.HANDOVER_FRAC)):
      unseen = self.stock.stop(-0.8, 60 / 3.6).band_gap(15, 9)
    self.assertGreater(seen, 0.1, f"the modelled dip at 15-9 km/h is {seen:+.3f}")
    self.assertLess(abs(unseen), 0.02, f"with the hand-over removed the dip should be gone, got {unseen:+.3f}")


class TestRestoredCorrections(unittest.TestCase):
  """What each restored brake-onset correction does, measured against stock.

  Both runs are the real carcontroller on the same plant and the same request, so the
  only difference is the correction. A correction is only worth having if it moves the
  metric it was written for; these are those before/after numbers. The friction entry
  is the one with a cost as well as a benefit, so both halves are asserted.
  """

  @staticmethod
  def _pair(request, v_kmh):
    """The same stop twice: (harness, trace) with the gate off and with it on."""
    stock, shaped = LongHarness(brake_onset=False), LongHarness(brake_onset=True)
    return (stock, stock.stop(request, v_kmh / 3.6)), (shaped, shaped.stop(request, v_kmh / 3.6))

  def test_handover_feedforward_fills_the_low_speed_dip(self):
    """The dip the driver reports as 'brakes, goes light, brakes again'."""
    (_, stock), (_, shaped) = self._pair(-0.8, 60)
    self.assertGreater(stock.band_gap(15, 9), 0.08, f"stock dip {stock.band_gap(15, 9):+.3f}")
    self.assertLess(shaped.band_gap(15, 9), 0.03, f"with the correction {shaped.band_gap(15, 9):+.3f}")
    self.assertLess(abs(shaped.band_gap(15, 9)), 0.5 * stock.band_gap(15, 9),
                    "the feed-forward should take out most of the dip, not merely some")

  def test_pid_hold_keeps_the_integrator_from_winding_up(self):
    """The wound-up integrator is what makes the last metres brake again."""
    (_, stock), (_, shaped) = self._pair(-0.8, 60)
    self.assertLess(abs(min(shaped.integral)), 0.5 * abs(min(stock.integral)),
                    f"integrator reached {min(shaped.integral):+.3f} against stock {min(stock.integral):+.3f}")

  def test_pid_hold_does_not_step_the_brake_at_low_speed(self):
    """The hold must not drop the over-delivery trim as a step (the low-speed jolt).

    While braking, the loop winds the integral positive to lift the command and match the
    PCM braking harder than asked (asked -2.0, delivered ~-2.15). The hold used to bleed
    that positive integral at PID_BLEED_POS, taking the whole trim back in one 2 m/s^3 step
    around 9 km/h - the brake suddenly goes firm. It must hold the trim instead.
    """
    (_, _), (_, shaped) = self._pair(-2.0, 48)
    spd = np.asarray(shaped.speed) * 3.6
    cmd = np.asarray(shaped.a_cmd)
    d = np.diff(cmd, prepend=cmd[0]) / LONG_DT
    m = (spd <= 9.5) & (spd > 5.0)
    self.assertGreater(d[m].min(), -1.0,
                       f"the command steps {d[m].min():.2f} m/s^3 at {spd[m][np.argmin(d[m])]:.1f} km/h")
    # the positive trim is what is held, and it still matches the request to the end
    self.assertGreater(shaped.integral[-1], 0.0)
    self.assertLess(abs(shaped.tracking_error(shaped.t[-1] - 1.0)), 0.1)

  def test_friction_entry_limit_softens_the_bite_and_costs_time(self):
    """Both halves matter: the entry is gentler, and a firm request arrives later."""
    (stock_h, stock), (shaped_h, shaped) = self._pair(-1.0, 60)
    self.assertLess(shaped_h.plant.entry_jerk, 0.5 * stock_h.plant.entry_jerk,
                    f"entry jolt {shaped_h.plant.entry_jerk:.2f} against stock {stock_h.plant.entry_jerk:.2f} m/s^3")
    rise, stock_rise = shaped.rise_time(), stock.rise_time()
    self.assertGreater(rise, stock_rise, f"rise time {rise:.2f}s against stock {stock_rise:.2f}s")
    self.assertLess(rise, 2.0, f"the cost has to be bounded, got {rise:.2f}s")

  def test_overshoot_limiter_takes_back_a_hard_request(self):
    """The car must not brake harder than openpilot asked on a hard request."""
    (_, stock), (_, shaped) = self._pair(-3.2, 80)
    self.assertGreater(stock.over_delivery(), 0.5, f"stock over-delivery {stock.over_delivery():+.3f}")
    self.assertLess(shaped.over_delivery(), stock.over_delivery() - 0.1,
                    f"with the limiter {shaped.over_delivery():+.3f}")
    self.assertGreater(shaped.peak_accel(), stock.peak_accel(),
                       f"peak decel {shaped.peak_accel():+.2f} against stock {stock.peak_accel():+.2f}")


class TestIntegratorLeak(unittest.TestCase):
  def test_persistence_deadband_stays_below_the_perceptible_threshold(self):
    """The leak sets a deadband of UNWIND / (k_i * i_rate) below which errors persist.

    Pin it, so raising the leak cannot silently reintroduce an uncorrected bias.
    """
    for v_ms in (2.0, 6.0, 30.0):
      with self.subTest(v_ms=v_ms):
        k_i = float(np.interp(v_ms, [2., 5.], [0.5, 0.25]))
        deadband = ACCEL_PID_UNWIND / (k_i * DT_CTRL * 3)
        self.assertLessEqual(deadband, 0.05,
                             f"persistent accel error up to {deadband:.3f} m/s^2 is never trimmed")


if __name__ == "__main__":
  unittest.main()
