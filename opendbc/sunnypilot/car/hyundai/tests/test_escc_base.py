import pytest
from hypothesis import given, strategies as st, settings, HealthCheck
from opendbc.sunnypilot.car.hyundai.escc import Escc
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


@pytest.fixture
def car_params():
  params = CarParams()
  params.carFingerprint = "HYUNDAI_SONATA"
  params.sunnypilotCarParams.flags = HyundaiFlagsSP.SP_ENHANCED_SCC.value
  return params


@pytest.fixture
def escc(car_params):
  return Escc(car_params)


class TestEscc:
  def test_escc_msg_id(self, escc):
    assert escc.ESCC_MSG_ID == 0x2AB

  @settings(suppress_health_check=[HealthCheck.function_scoped_fixture])
  @given(st.integers(min_value=0, max_value=255))
  def test_enabled_flag(self, car_params, value):
    car_params.sunnypilotCarParams.flags = value
    escc = Escc(car_params)
    assert escc.enabled == (value & HyundaiFlagsSP.SP_ENHANCED_SCC.value)

  def test_refresh_car_state(self, escc, car_params):
    car_state = CarState(car_params)
    car_state.escc_cmd_act = 1
    car_state.escc_aeb_warning = 1
    car_state.escc_aeb_dec_cmd_act = 1
    car_state.escc_aeb_dec_cmd = 1
    escc.refresh_car_state(car_state)
    assert escc.car_state == car_state

  def test_update_scc12_message(self, escc, car_params):
    car_state = CarState(car_params)
    car_state.escc_cmd_act = 1
    car_state.escc_aeb_warning = 1
    car_state.escc_aeb_dec_cmd_act = 1
    car_state.escc_aeb_dec_cmd = 1
    escc.refresh_car_state(car_state)
    scc12_message = {}
    escc.update_scc12_message(scc12_message)
    assert scc12_message["AEB_CmdAct"] == 1
    assert scc12_message["CF_VSM_Warn"] == 1
    assert scc12_message["CF_VSM_DecCmdAct"] == 1
    assert scc12_message["CR_VSM_DecCmd"] == 1
    assert scc12_message["AEB_Status"] == 2

  def test_get_radar_escc_parser(self, escc):
    parser = escc.get_radar_escc_parser()
    assert parser is not None
    assert parser.dbc_name == b"hyundai_kia_generic"
