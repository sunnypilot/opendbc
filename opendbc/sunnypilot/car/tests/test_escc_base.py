import pytest
from hypothesis import given, strategies as st, settings, HealthCheck
from opendbc.sunnypilot.car.escc_base import EsccBase
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.flags import HyundaiFlagsSP
from opendbc.can.parser import CANParser

class TestEsccBase(EsccBase):
  @property
  def ESCC_MSG_ID(self):
    return 0x2AB

  def get_radar_escc_parser(self) -> CANParser:
    pass

@pytest.fixture
def car_params():
  params = CarParams()
  params.carFingerprint = "HYUNDAI_SONATA"
  params.flags = HyundaiFlagsSP.SP_ENHANCED_SCC.value
  return params

@pytest.fixture
def escc_base(car_params):
  return TestEsccBase(car_params)

class TestEsccBaseClass:
  def test_escc_msg_id(self, escc_base):
    assert escc_base.ESCC_MSG_ID == 0x2AB

  @settings(suppress_health_check=[HealthCheck.function_scoped_fixture])
  @given(st.integers(min_value=0, max_value=255))
  def test_enabled_flag(self, car_params, value):
    car_params.flags = value
    escc_base = TestEsccBase(car_params)
    assert escc_base.enabled == (value & HyundaiFlagsSP.SP_ENHANCED_SCC.value)
