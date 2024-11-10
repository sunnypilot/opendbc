from abc import ABC, abstractmethod
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.hyundai.flags import HyundaiFlagsSP
from opendbc.car.structs import CarParams


class EsccBase(ABC):
  def __init__(self, car_params: CarParams):
    self.car_params = car_params

  @property
  def enabled(self):
    return self.car_params.sunnyParams.flags & HyundaiFlagsSP.SP_ENHANCED_SCC.value

  @property
  @abstractmethod
  def ESCC_MSG_ID(self):
    raise NotImplementedError

  @abstractmethod
  def get_radar_escc_parser(self) -> CANParser:
    raise NotImplementedError
