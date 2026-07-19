import unittest
from unittest.mock import MagicMock
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.subaru.values import DBC, CAR, CanBus
from opendbc.car.subaru import subarucan
from opendbc.sunnypilot.car.subaru import subarucan_ext


def make_packer():
  packer = MagicMock()
  packer.make_can_msg.return_value = ("ES_Brake", b'\x00' * 8, 0)
  return packer


def make_es_brake_msg(**overrides):
  base = {
    "CHECKSUM": 0,
    "Signal1": 0,
    "Brake_Pressure": 0,
    "AEB_Status": 0,
    "Cruise_Brake_Lights": 0,
    "Cruise_Brake_Fault": 0,
    "Cruise_Brake_Active": 0,
    "Cruise_Activated": 0,
    "Signal3": 0,
  }
  base.update(overrides)
  return base


def get_values(packer):
  _, _, values = packer.make_can_msg.call_args[0]
  return values


def make_brake_status_msg(**overrides):
  base = {"CHECKSUM": 0, "COUNTER": 0, "Signal1": 0, "ES_Brake": 0, "Signal2": 0, "Brake": 0, "Signal3": 0}
  base.update(overrides)
  return base


class TestSubarucanBrakeHold(unittest.TestCase):
  # --- Brake_Pressure / Cruise_Brake_Active / Cruise_Brake_Lights ---

  def test_brake_field_packing(self):
    for brake_value, active, lights in [
      (0, False, False),
      (50, True, False),   # active >0, below 70-unit lights threshold
      (70, True, True),    # lights threshold is inclusive
      (100, True, True),
      (600, True, True),
    ]:
      with self.subTest(brake_value=brake_value):
        packer = make_packer()
        subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=brake_value)
        v = get_values(packer)
        self.assertEqual(v["Brake_Pressure"], brake_value)
        self.assertEqual(bool(v["Cruise_Brake_Active"]), active)
        self.assertEqual(bool(v["Cruise_Brake_Lights"]), lights)

  def test_stock_path_packs_brake_fields(self):
    # long_enabled=False still derives Active/Lights from brake_value; only fault handling differs
    packer = make_packer()
    subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=False, brake_value=100)
    v = get_values(packer)
    self.assertEqual(v["Brake_Pressure"], 100)
    self.assertTrue(v["Cruise_Brake_Active"])
    self.assertTrue(v["Cruise_Brake_Lights"])

  # --- Cruise_Brake_Fault: cleared on op-long, forwarded on stock ---

  def test_cruise_brake_fault_cleared_when_long_enabled(self):
    # Eyesight latches Cruise_Brake_Fault=1 after op-long ACC engages; forwarding it poisoned the
    # AVH hold (car rolled), so op-long forces it to 0. Route dde08cad3a74cd94/00000014--7d3ff4c569.
    packer = make_packer()
    subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Brake_Fault=1), long_enabled=True, brake_value=100)
    self.assertEqual(get_values(packer)["Cruise_Brake_Fault"], 0)

  def test_cruise_brake_fault_forwarded_on_stock(self):
    # stock Eyesight ACC: the fault is genuinely Eyesight's, forward verbatim
    packer = make_packer()
    subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Brake_Fault=1), long_enabled=False, brake_value=100)
    self.assertEqual(get_values(packer)["Cruise_Brake_Fault"], 1)

  def test_hold_pressure_with_eyesight_fault_latched(self):
    # regression: 600-unit hold must still go out with fault cleared when Eyesight has latched the fault
    packer = make_packer()
    subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Brake_Fault=1), long_enabled=True, brake_value=600)
    v = get_values(packer)
    self.assertEqual(v["Brake_Pressure"], 600)
    self.assertTrue(v["Cruise_Brake_Active"])
    self.assertEqual(v["Cruise_Brake_Fault"], 0)

  # --- Passthrough fields ---

  def test_cruise_activated_passthrough(self):
    # forwarded verbatim — NOT overridden, unlike create_es_brake
    packer = make_packer()
    subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Activated=1), long_enabled=True, brake_value=100)
    self.assertEqual(get_values(packer)["Cruise_Activated"], 1)

  def test_signal_passthrough(self):
    for signal, value in [("AEB_Status", 8), ("Signal1", 5), ("Signal3", 3)]:
      with self.subTest(signal=signal):
        packer = make_packer()
        subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(**{signal: value}), long_enabled=True, brake_value=0)
        self.assertEqual(get_values(packer)[signal], value)

  # --- COUNTER ---

  def test_counter(self):
    for frame, counter in [(0, 0), (15, 15), (16, 0)]:  # 16 % 16 == 0 wraps
      with self.subTest(frame=frame):
        packer = make_packer()
        subarucan.create_es_brake_hold(packer, frame=frame, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=0)
        self.assertEqual(get_values(packer)["COUNTER"], counter)

  # --- Real CANPacker smoke test (catches DBC signal-name drift; also covers msg name + bus) ---

  def test_real_packer_accepts_signals(self):
    packer = CANPacker(DBC[CAR.SUBARU_IMPREZA_2020.value][Bus.pt])
    addr, dat, bus = subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(),
                                                    long_enabled=True, brake_value=600)
    self.assertEqual(addr, 0x220)
    self.assertEqual(bus, CanBus.main)
    self.assertEqual(len(dat), 8)

  # --- Brake_Status ES_Brake bit must stay at byte 7 bit 2 (panda reads `(data[7] >> 2) & 1`) ---

  def test_brake_status_es_brake_bit_offset_matches_safety(self):
    # Pin the DBC ES_Brake location to the hardcoded panda offset; a DBC move would silently
    # desync subaru_tx_hook's Brake_Status check from create_brake_status_hold's packing.
    packer = CANPacker(DBC[CAR.SUBARU_IMPREZA_2020.value][Bus.pt])
    _, dat_set, _ = packer.make_can_msg("Brake_Status", CanBus.camera, make_brake_status_msg(ES_Brake=1))
    self.assertEqual(dat_set[7], 0x04)  # only ES_Brake set -> byte 7 bit 2

    # the hold mask clears that exact bit while forwarding Brake (bit 6)
    _, dat_hold, bus = subarucan_ext.create_brake_status_hold(packer, make_brake_status_msg(ES_Brake=1, Brake=1))
    self.assertEqual((dat_hold[7] >> 2) & 1, 0)
    self.assertEqual((dat_hold[7] >> 6) & 1, 1)
    self.assertEqual(bus, CanBus.camera)


if __name__ == "__main__":
  unittest.main()
