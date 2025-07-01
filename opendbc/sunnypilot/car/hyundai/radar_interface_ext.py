import math

from opendbc.can.parser import CANParser
from opendbc.car import structs, Bus
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.escc import EsccRadarInterfaceBase
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


class RadarInterfaceExt(EsccRadarInterfaceBase):
  msg_src: str
  trigger_msg: int
  rcp: CANParser
  pts: dict[int, structs.RadarData.RadarPoint]

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    EsccRadarInterfaceBase.__init__(self, CP, CP_SP)
    self.CP = CP
    self.CP_SP = CP_SP

    self.track_id = 0

  @property
  def use_radar_interface_ext(self) -> bool:
    return self.use_escc or self.CP.flags & (HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC)

  def get_msg_src(self) -> str | None:
    if self.use_escc:
      return "ESCC"
    if self.CP.flags & (HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC):
      return "SCC_CONTROL" if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else "SCC11"

  def get_radar_ext_can_parser(self) -> CANParser:
    if self.ESCC.enabled:
      lead_src, bus = "ESCC", 0
    elif self.CP.flags & (HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC):
      lead_src = "SCC_CONTROL" if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else "SCC11"
      bus = CanBus(self.CP).CAM if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else 2
    else:
      return None

    messages = [(lead_src, 50)]
    return CANParser(DBC[self.CP.carFingerprint][Bus.pt], messages, bus)

  def get_trigger_msg(self, default_trigger_msg) -> int:
    if self.ESCC.enabled:
      return self.ESCC.trigger_msg
    if self.CP.flags & (HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC):
      return 0x1A0 if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else 0x420
    return default_trigger_msg

  def initialize_radar_ext(self, default_trigger_msg) -> None:
    if self.ESCC.enabled:
      self.use_escc = True

    self.rcp = self.get_radar_ext_can_parser()
    self.trigger_msg = self.get_trigger_msg(default_trigger_msg)

  def update_ext(self, ret: structs.RadarData) -> structs.RadarData:
    if not self.rcp.can_valid:
      ret.errors.canError = True
      return ret

    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY:
      print("RADAR_LEAD_ONLY")
      for ii in range(1):
        msg_src = self.get_msg_src()
        msg = self.rcp.vl[msg_src]

        if ii not in self.pts:
          self.pts[ii] = structs.RadarData.RadarPoint()
          self.pts[ii].trackId = self.track_id
          self.track_id += 1

        valid = msg['ACC_ObjDist'] < 204.6 if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else msg['ACC_ObjStatus']
        if valid:
          self.pts[ii].measured = True
          self.pts[ii].dRel = msg['ACC_ObjDist']
          self.pts[ii].yRel = float('nan')  # FIXME-SP: Only some cars have lateral position from SCC
          self.pts[ii].vRel = msg['ACC_ObjRelSpd']
          self.pts[ii].aRel = float('nan')  # TODO-SP: calculate from ACC_ObjRelSpd and with timestep 50Hz (needs to modify in interfaces.py)
          self.pts[ii].yvRel = float('nan')

        else:
          del self.pts[ii]

    elif self.CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR:
      print("RADAR_FULL_RADAR")
      for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):

        if self.CP_flags & HyundaiFlags.MRREVO14F_RADAR:
          msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
          addr1 = f"{addr}_1"
          addr2 = f"{addr}_2"

          if addr1 not in self.pts:
            self.pts[addr1] = structs.RadarData.RadarPoint()
            self.pts[addr1].trackId = self.track_id
            self.track_id += 1

          valid = msg['1_DISTANCE'] != 255.75
          if valid:
            self.pts[addr1].measured = True
            self.pts[addr1].dRel = msg['1_DISTANCE']
            self.pts[addr1].yRel = msg['1_LATERAL']
            self.pts[addr1].vRel = float('nan')
            self.pts[addr1].aRel = float('nan')
            self.pts[addr1].yvRel = float('nan')
          else:
            del self.pts[addr1]

          if addr2 not in self.pts:
            self.pts[addr2] = structs.RadarData.RadarPoint()
            self.pts[addr2].trackId = self.track_id
            self.track_id += 1

          valid = msg['2_DISTANCE'] != 255.75
          if valid:
            self.pts[addr2].measured = True
            self.pts[addr2].dRel = msg['2_DISTANCE']
            self.pts[addr2].yRel = msg['2_LATERAL']
            self.pts[addr2].vRel = float('nan')
            self.pts[addr2].aRel = float('nan')
            self.pts[addr2].yvRel = float('nan')
          else:
            del self.pts[addr2]

        else:
          msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

          if addr not in self.pts:
            self.pts[addr] = structs.RadarData.RadarPoint()
            self.pts[addr].trackId = self.track_id
            self.track_id += 1

          valid = msg['STATE'] in (3, 4)
          if valid:
            azimuth = math.radians(msg['AZIMUTH'])
            self.pts[addr].measured = True
            self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
            self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
            self.pts[addr].vRel = msg['REL_SPEED']
            self.pts[addr].aRel = msg['REL_ACCEL']
            self.pts[addr].yvRel = float('nan')
          else:
            del self.pts[addr]

    elif self.CP_SP.flags & HyundaiFlagsSP.RADAR_OFF:
      print("RADAR_OFF")

    else:
      print("RADAR_ERROR")

    ret.points = list(self.pts.values())
    return ret
