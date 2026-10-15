import unittest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR

ButtonType = structs.CarState.ButtonEvent.Type


class TestMazdaMainOnButton(unittest.TestCase):
  def setUp(self):
    fingerprint = gen_empty_fingerprint()
    cp = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=False, is_release=False, docs=False)
    cp_sp = CarInterface.get_params_sp(cp, CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
    self.interface = CarInterface(cp, cp_sp)
    self.packer = CANPacker('mazda_2017')
    self.frame = 0

  def _update(self, payload):
    self.frame += 1
    cs, _ = self.interface.update([(self.frame * 100_000_000, [(0x9d, payload, 0)])])
    return [(str(be.type), be.pressed) for be in cs.buttonEvents]

  def test_recorded_on_press_hold_and_release(self):
    # Recorded CX-5 2022 physical ON: MODE_X=MODE_Y=1.
    idle = bytes.fromhex('0001fff000000000')
    on = bytes.fromhex('0061ff0800000000')
    self.assertEqual([], self._update(idle))
    self.assertEqual([('mainCruiseOn', True)], self._update(on))
    self.assertEqual([], self._update(on))
    self.assertEqual([('mainCruiseOn', False)], self._update(idle))

  def test_mode_y_alone_is_on(self):
    self._update(self.packer.make_can_msg('CRZ_BTNS', 0, {})[1])
    payload = self.packer.make_can_msg('CRZ_BTNS', 0, {'MODE_Y': 1})[1]
    self.assertEqual([('mainCruiseOn', True)], self._update(payload))

  def test_off_cancel_and_tja_are_not_on(self):
    # Recorded OFF is CAN_OFF; the KE can also send MODE_X alone for OFF.
    for payload in (bytes.fromhex('0101fdf000000000'),
                    self.packer.make_can_msg('CRZ_BTNS', 0, {'MODE_X': 1})[1],
                    self.packer.make_can_msg('CRZ_BTNS', 0, {'TJA_BUTTON': 1})[1]):
      with self.subTest(payload=payload.hex()):
        self._update(self.packer.make_can_msg('CRZ_BTNS', 0, {})[1])
        self.assertEqual([], self._update(payload))


if __name__ == '__main__':
  unittest.main()
