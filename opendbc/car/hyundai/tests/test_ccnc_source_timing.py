import copy
from types import SimpleNamespace
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags


DBC = "hyundai_canfd_generated"

# Stock frames from route 428c22dc6061cee7|00000115--9f58c41b0b.
STOCK_161 = "b7cce84000000000c0fff0c003000040000000000000000000ff000000000100"
STOCK_162 = "c9f3e82100000000c0ff00000000000000000000000000000000000802040000"


def decode(name, address, data):
  parser = CANParser(DBC, [(name, 20)], 2)
  parser.update([(1, [(address, bytes.fromhex(data), 2)])])
  return copy.copy(parser.vl[name])


class TestCcncSourceTiming(unittest.TestCase):
  def setUp(self):
    self.msg_161 = decode("CCNC_0x161", 0x161, STOCK_161)
    self.msg_162 = decode("CCNC_0x162", 0x162, STOCK_162)
    self.hud = SimpleNamespace(
      leftLaneVisible=True,
      rightLaneVisible=True,
      leftLaneDepart=False,
      rightLaneDepart=False,
      leadDistanceBars=3,
      leadVisible=False,
    )
    self.cs = SimpleNamespace(
      ccnc_0x161_updated=False,
      ccnc_0x162_updated=False,
      msg_161=copy.copy(self.msg_161),
      msg_162=copy.copy(self.msg_162),
      msg_1b5={
        "Info_LftLnPosVal": 1.0,
        "Info_RtLnPosVal": 2.0,
        "Info_LftLnQualSta": 3,
        "Info_RtLnQualSta": 3,
        "Longitudinal_Distance": 22.6,
      },
      is_metric=True,
      out=SimpleNamespace(
        steeringAngleDeg=0.0,
        vEgo=20.0,
        leftBlindspot=False,
        rightBlindspot=False,
        vCruiseCluster=0.0,
      ),
      main_cruise_enabled=False,
      buttons_counter=0,
      cruise_info={},
    )
    self.cc = SimpleNamespace(
      enabled=False,
      hudControl=self.hud,
      leftBlinker=False,
      rightBlinker=False,
      cruiseControl=SimpleNamespace(override=False, cancel=False, resume=False),
    )

    self.controller = CarController.__new__(CarController)
    self.controller.CP = SimpleNamespace(flags=HyundaiFlags.CCNC, openpilotLongitudinalControl=False)
    self.controller.CAN = CanBus(None, fingerprint={})
    self.controller.packer = CANPacker(DBC)
    self.controller.frame = 0
    self.controller.lkas_icon = 0
    self.controller.lfa_icon = 2
    self.controller.last_button_frame = 0

  def display_messages(self, frame, updated_161=False, updated_162=False):
    self.controller.frame = frame
    self.cs.ccnc_0x161_updated = updated_161
    self.cs.ccnc_0x162_updated = updated_162
    messages = self.controller.create_canfd_msgs(True, 20, 0.0, 0.0, False, self.hud, self.cs, self.cc)
    return [msg for msg in messages if msg[0] in (0x161, 0x162)]

  def test_sends_only_the_source_message_updated_this_control_cycle(self):
    self.assertEqual([msg[0] for msg in self.display_messages(0, updated_161=True)], [0x161])
    self.assertEqual(self.display_messages(1), [])
    self.assertEqual([msg[0] for msg in self.display_messages(2, updated_162=True)], [0x162])
    self.assertEqual(self.display_messages(5), [])
    self.assertEqual([msg[0] for msg in self.display_messages(6, updated_161=True, updated_162=True)], [0x161, 0x162])

  def test_preserves_stock_phase_and_source_counters(self):
    expected = [(0, 0x161, 10), (2, 0x162, 40), (5, 0x161, 11), (7, 0x162, 41)]
    actual = []

    for frame, address, counter in expected:
      if address == 0x161:
        self.cs.msg_161 = copy.copy(self.msg_161)
        self.cs.msg_161["COUNTER"] = counter
      else:
        self.cs.msg_162 = copy.copy(self.msg_162)
        self.cs.msg_162["COUNTER"] = counter

      for msg_address, data, _ in self.display_messages(frame, address == 0x161, address == 0x162):
        name = "CCNC_0x161" if msg_address == 0x161 else "CCNC_0x162"
        actual.append((frame, msg_address, int(decode(name, msg_address, data.hex())["COUNTER"])))

    self.assertEqual(actual, expected)


if __name__ == "__main__":
  unittest.main()
