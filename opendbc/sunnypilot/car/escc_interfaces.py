from abc import ABC, abstractmethod

from opendbc.car import CanData
from opendbc.car.interfaces import CarStateBase
from opendbc.car import structs


class EsccStateBase(ABC):
  def __init__(self):
    self.escc_aeb_warning = 0
    self.escc_aeb_dec_cmd_act = 0
    self.escc_cmd_act = 0
    self.escc_aeb_dec_cmd = 0


class EsccControllerBase(ABC):
  def __init__(self, CP):
    self.CP = CP
    self.ESCC = None
    self.init()
    if self.ESCC is None:
      raise ValueError("ESCC is not initialized")

  @abstractmethod
  def init(self):
    raise NotImplementedError

  @abstractmethod
  def update(self, CC: structs.CarControl, CS: CarStateBase, now_nanos: int) -> tuple[structs.CarControl.Actuators, list[CanData]]:
    raise NotImplementedError
