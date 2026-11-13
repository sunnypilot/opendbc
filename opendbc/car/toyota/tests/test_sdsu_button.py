import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, CanData, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.values import CAR, DBC

GAP = structs.CarState.ButtonEvent.Type.gapAdjustCruise


def make_interface(car, fingerprint):
  fp = {b: {} for b in range(4)}
  fp[0] = fingerprint
  car_interface = interfaces[car]
  cp = car_interface.get_params(car, fp, [], alpha_long=True, is_release=False, docs=False)
  cp_sp = car_interface.get_params_sp(cp, car, fp, [], alpha_long=True, is_release_sp=False, docs=False)
  return car_interface(cp, cp_sp)


def gap_events(ci, packer, fd_button_frames):
  events = []
  for i, pressed in enumerate(fd_button_frames):
    msg = packer.make_can_msg("SDSU", 0, {"FD_BUTTON": pressed, "STATE": 0})
    cs, _ = ci.update([(i * 10_000_000, [CanData(msg[0], msg[1], 0)])])
    events += [e.pressed for e in cs.buttonEvents if e.type == GAP]
  return events


class TestSdsuFollowDistanceButton(unittest.TestCase):
  PRESS = [0] * 3 + [1] * 5 + [0] * 3

  def setUp(self):
    self.packer = CANPacker(DBC[CAR.TOYOTA_RAV4_TSS2_2022][Bus.pt])

  def test_radar_acc_with_smartdsu_reports_button(self):
    # a radar CAN filter forwards the wheel button on its 0x2FF heartbeat; radar-ACC cars have no other path for it
    ci = make_interface(CAR.TOYOTA_RAV4_TSS2_2022, {0x2FF: 4})
    assert ci.CP.flags & 64 and ci.CP_SP.flags & 1  # RADAR_ACC, SMART_DSU
    assert gap_events(ci, self.packer, self.PRESS) == [True, False]

  def test_radar_acc_without_smartdsu_ignores_sdsu_frame(self):
    ci = make_interface(CAR.TOYOTA_RAV4_TSS2_2022, {})
    assert gap_events(ci, self.packer, self.PRESS) == []

  def test_radar_can_filter_flag_skips_sdsu(self):
    # a 0x2AA radar CAN filter has no SDSU heartbeat to read
    ci = make_interface(CAR.TOYOTA_RAV4_TSS2_2022, {0x2FF: 4, 0x2AA: 8})
    if ci.CP_SP.flags & 2:  # RADAR_CAN_FILTER
      assert gap_events(ci, self.packer, self.PRESS) == []


if __name__ == "__main__":
  unittest.main()
