from opendbc.car import structs
from opendbc.car.toyota.values import ToyotaFlags
from opendbc.sunnypilot.car.toyota.secoc_long import SecOCLong
import pytest
from hypothesis import given, strategies as st, settings, HealthCheck
from opendbc.car.toyota.carstate import CarState


@pytest.fixture
def car_params():
  params = structs.CarParams()
  params.carFingerprint = "TOYOTA_RAV4_PRIME"
  return params


@pytest.fixture
def car_params_sp():
  params = structs.CarParams()
  params.flags = ToyotaFlags.SECOC.value
  return params


@pytest.fixture
def secoc_long(car_params):
  return SecOCLong(car_params)


class TestSecOCLong:
  @settings(suppress_health_check=[HealthCheck.function_scoped_fixture])
  @given(st.integers(min_value=0, max_value=255))
  def test_enabled_flag(self, car_params, value):
    car_params.flags = value
    secoc_long = SecOCLong(car_params)
    assert secoc_long.enabled == (value & ToyotaFlags.SECOC.value)

  def test_update_car_state(self, secoc_long, car_params, car_params_sp):
    car_state = CarState(car_params, car_params_sp)
    car_state.secoc_key = b"22" * 16
    car_state.secoc_synchronization = {
      'TRIP_CNT': 1,
      'RESET_CNT': 1
    }
    secoc_long.update_car_state(car_state)
    assert secoc_long.car_state == car_state
