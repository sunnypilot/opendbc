import unittest

from opendbc.can.parser import CANParser
from opendbc.car import create_button_events, structs
from opendbc.car.toyota.carstate import get_virtual_cruise_button, VIRTUAL_CRUISE_BUTTONS
from opendbc.car.toyota.interface import CarInterface
from opendbc.car.toyota.values import CAR
from opendbc.sunnypilot.car.interfaces import _initialize_toyota
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP
from opendbc.testing import parameterized


def build_params(*, platform=CAR.TOYOTA_COROLLA_TSS2, available=True, openpilot_long=True):
  CP = structs.CarParams(
    brand="toyota",
    carFingerprint=str(platform),
    openpilotLongitudinalControl=openpilot_long,
    alphaLongitudinalAvailable=True,
    safetyConfigs=[structs.CarParams.SafetyConfig()],
  )
  flags = ToyotaFlagsSP.VIRTUAL_CRUISE_SPEED_AVAILABLE if available else 0
  CP_SP = structs.CarParamsSP(flags=flags, pcmCruiseSpeed=True)
  return CP, CP_SP


class TestVirtualCruiseSpeed(unittest.TestCase):
  @parameterized("payload, expected_res, expected_set, expected_button", (
    ("861a0000561b1a81", 0, 0, 0),
    ("a61a0000561a1a81", 1, 0, 1),
    ("961e0000561f1f83", 0, 1, 2),
    ("a61b0000561c1c80", 1, 0, 1),
    ("965f000056666585", 0, 1, 2),
  ))
  def test_route_cruise_button_signals(self, payload, expected_res, expected_set, expected_button):
    parser = CANParser("toyota_nodsu_pt_generated", [("CLUTCH", 16)], 0)
    parser.update((1, [(0x361, bytes.fromhex(payload), 0)]))

    cruise_res = int(parser.vl["CLUTCH"]["CRUISE_RES"])
    cruise_set = int(parser.vl["CLUTCH"]["CRUISE_SET"])
    self.assertEqual((cruise_res, cruise_set), (expected_res, expected_set))
    self.assertEqual(get_virtual_cruise_button(cruise_res, cruise_set), expected_button)

  def test_simultaneous_cruise_buttons_map_to_unpressed(self):
    self.assertEqual(get_virtual_cruise_button(1, 1), 0)

  @parameterized("previous, current, expected", (
    (0, 1, ((structs.CarState.ButtonEvent.Type.accelCruise, True),)),
    (1, 0, ((structs.CarState.ButtonEvent.Type.accelCruise, False),)),
    (1, 2, (
      (structs.CarState.ButtonEvent.Type.accelCruise, False),
      (structs.CarState.ButtonEvent.Type.decelCruise, True),
    )),
  ))
  def test_virtual_cruise_button_edges(self, previous, current, expected):
    events = create_button_events(current, previous, VIRTUAL_CRUISE_BUTTONS)
    self.assertEqual(tuple((event.type, event.pressed) for event in events), expected)

  def test_virtual_cruise_speed_capability_is_platform_scoped(self):
    for candidate, expected in (
      (CAR.TOYOTA_COROLLA_TSS2, True),
      (CAR.TOYOTA_PRIUS_TSS2, True),
      (CAR.TOYOTA_RAV4_TSS2, False),
    ):
      with self.subTest(candidate=candidate):
        CP, CP_SP = build_params(platform=candidate, available=False)
        CarInterface._get_params_sp(CP, CP_SP, candidate, {0: {}}, [], False, False, False)
        self.assertEqual(bool(CP_SP.flags & ToyotaFlagsSP.VIRTUAL_CRUISE_SPEED_AVAILABLE), expected)

  @parameterized("enabled, available, openpilot_long, expected_pcm_speed", (
    (True, True, True, False),
    (False, True, True, True),
    (True, False, True, True),
    (True, True, False, True),
  ))
  def test_virtual_cruise_speed_ownership(self, enabled, available, openpilot_long, expected_pcm_speed):
    CP, CP_SP = build_params(available=available, openpilot_long=openpilot_long)
    _initialize_toyota(CP, CP_SP, {"ToyotaVirtualCruiseSpeed": int(enabled)})
    self.assertEqual(CP_SP.pcmCruiseSpeed, expected_pcm_speed)

  def test_stock_longitudinal_wins_over_virtual_cruise_speed(self):
    CP, CP_SP = build_params()
    _initialize_toyota(CP, CP_SP, {
      "ToyotaEnforceStockLongitudinal": 1,
      "ToyotaVirtualCruiseSpeed": 1,
    })

    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertTrue(CP_SP.pcmCruiseSpeed)


if __name__ == "__main__":
  unittest.main()
