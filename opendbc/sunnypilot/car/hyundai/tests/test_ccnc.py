"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from opendbc.sunnypilot.car.hyundai.ccnc import create_ccnc_0x161

ALERTS = ("ALERTS_1", "ALERTS_2", "ALERTS_3", "ALERTS_4", "ALERTS_5", "SOUNDS_1", "SOUNDS_2", "SOUNDS_3", "SOUNDS_4")


class Packer:
  @staticmethod
  def make_can_msg(name, bus, values):
    return values


def build(msg_161, long_enabled=True, interceptor=True):
  hud = SimpleNamespace(leadDistanceBars=2)
  return create_ccnc_0x161(Packer, SimpleNamespace(ECAN=0), 0, msg_161, True, True, False, long_enabled, True, False, hud,
                           60, 20.0, True, False, interceptor)


def dcu_161(**kw):
  return {**dict.fromkeys(ALERTS, 0), **kw}


class TestCcncDcuAlerts:
  def test_cruise_popups_dropped(self):
    for msg in (dcu_161(ALERTS_5=4, SOUNDS_3=5), dcu_161(ALERTS_3=9, SOUNDS_4=2), dcu_161(ALERTS_5=3), dcu_161(ALERTS_4=2),
                dcu_161(ALERTS_2=4, SOUNDS_2=2)):
      out = build(msg)
      assert all(out[k] == 0 for k in ALERTS), msg

  def test_safety_alerts_kept(self):
    for msg in (dcu_161(ALERTS_1=11, SOUNDS_1=6), dcu_161(ALERTS_2=9, SOUNDS_2=3), dcu_161(ALERTS_1=1, ALERTS_5=4, SOUNDS_1=3)):
      out = build(msg)
      for k in ("ALERTS_1", "ALERTS_2", "SOUNDS_1", "SOUNDS_2"):
        assert out[k] == msg[k], (msg, k)

  def test_untouched_without_interceptor_long(self):
    msg = dcu_161(ALERTS_5=4, ALERTS_3=9, SOUNDS_3=5)
    for out in (build(msg, interceptor=False), build(msg, long_enabled=False)):
      assert all(out[k] == msg[k] for k in ALERTS)
