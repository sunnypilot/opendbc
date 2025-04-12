from parameterized import parameterized

from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.hyundai.escc import ESCC_MSG
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces

TEST_CARS = [
  (CAR.HYUNDAI_ELANTRA_2021, ESCC_MSG),
  (CAR.HYUNDAI_ELANTRA_2021, None),
]



class TestRadarInterfaceExt:

  @staticmethod
  def _setup_platform(car_name):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
    CI = CarInterface(CP, CP_SP)

    sunnypilot_interfaces.setup_interfaces(CP, CP_SP)
    assert CP
    assert CP_SP
    assert CI

    return CI, CP, CP_SP

  @parameterized.expand(TEST_CARS)
  def test_radar_interface_ext(self, car_name, trigger_msg):
    CI, CP, CP_SP = self._setup_platform(car_name)

    RD = CI.RadarInterface(CP, CP_SP)
    assert RD
