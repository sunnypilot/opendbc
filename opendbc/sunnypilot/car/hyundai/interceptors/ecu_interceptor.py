from opendbc.car import structs
from abc import ABC, abstractmethod
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP

class EcuInterceptorBase(ABC):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.car_state = None

  @property
  def available(self):
    return self.CP_SP.flags & HyundaiFlagsSP.ADAS_ECU_INTERCEPTOR

  @property
  @abstractmethod
  def trigger_msg(self):
    pass

  def update_car_state(self, car_state):
    """
      This method is invoked by the CarController to update the car state on the ESCC object.
      The updated state is then used to update SCC12 with the current car state values received through ESCC.
      :param car_state:
      :return:
    """
    self.car_state = car_state

class EcuInterceptorCarStateBase(ABC):
  def __init__(self):
    pass


class EcuInterceptorCarControllerBase(ABC):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.interceptor = self.initialize(CP, CP_SP)

  @abstractmethod
  def initialize(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> EcuInterceptorBase:
    pass

  def update(self, car_state):
    self.interceptor.update_car_state(car_state)


class EcuInterceptorRadarInterfaceBase(ABC):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.interceptor = self.initialize(CP, CP_SP)
    self.use_interceptor = False

  @abstractmethod
  def initialize(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> EcuInterceptorBase:
    """
      This method is invoked to initialize the ESCC object.
      It should be called before any other methods are invoked.
    """
