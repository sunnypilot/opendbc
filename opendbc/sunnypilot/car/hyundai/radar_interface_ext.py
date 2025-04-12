from opendbc.can.parser import CANParser
from opendbc.car import structs, Bus
from opendbc.car.hyundai.values import DBC

from opendbc.sunnypilot.car.hyundai.escc import EsccRadarInterfaceBase


class RadarInterfaceExt(EsccRadarInterfaceBase):
  msg_src: str
  rcp: CANParser
  pts: dict[int, structs.RadarData.RadarPoint]

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    EsccRadarInterfaceBase.__init__(self, CP, CP_SP)
    self.CP = CP
    self.CP_SP = CP_SP

    self.track_id = 0

  def initialize_radar_ext(self, trigger_msg):
    if self.ESCC.enabled:
      self.use_escc = True

    rcp, trigger_msg = self.get_radar_ext_can_parser(trigger_msg)
    return rcp, trigger_msg

  def get_radar_ext_can_parser(self, trigger_msg):
    if self.ESCC.enabled:
      trigger_msg = self.ESCC.trigger_msg
      lead_src, bus = "ESCC", 0
    else:
      return None, trigger_msg
    messages = [(lead_src, 50)]
    return CANParser(DBC[self.CP.carFingerprint][Bus.pt], messages, bus), trigger_msg

  def get_msg_src(self):
    if self.use_escc:
      return "ESCC"

  def use_radar_interface_ext(self) -> bool:
    return self.use_escc

  def update_ext(self, ret):
    for ii in range(1):
      msg_src = self.get_msg_src()
      msg = self.rcp.vl[msg_src]

      if ii not in self.pts:
        self.pts[ii] = structs.RadarData.RadarPoint()
        self.pts[ii].trackId = self.track_id
        self.track_id += 1

      valid = msg['ACC_ObjStatus']
      if valid:
        self.pts[ii].measured = True
        self.pts[ii].dRel = msg['ACC_ObjDist']
        self.pts[ii].yRel = float('nan')  # FIXME-SP: Only some cars have lateral position from SCC
        self.pts[ii].vRel = msg['ACC_ObjRelSpd']
        self.pts[ii].aRel = float('nan')  # TODO-SP: calculate from ACC_ObjRelSpd and with timestep 50Hz (needs to modify in interfaces.py)
        self.pts[ii].yvRel = float('nan')

      else:
        del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret
