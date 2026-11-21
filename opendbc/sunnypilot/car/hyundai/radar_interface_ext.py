"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.can.parser import CANParser
from opendbc.car import structs, Bus
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.escc import EsccRadarInterfaceBase

RADAR_TRACKS_START_ADDR = 0x3A5
RADAR_TRACKS_MSG_COUNT = 32
RADAR_TRACKS_MSG_SIZE = 24
RADAR_TRACKS_ADDRS = range(RADAR_TRACKS_START_ADDR, RADAR_TRACKS_START_ADDR + RADAR_TRACKS_MSG_COUNT)


def has_radar_tracks(fingerprint: dict[int, dict[int, int]], bus: int) -> bool:
  return all(fingerprint[bus].get(addr) == RADAR_TRACKS_MSG_SIZE for addr in RADAR_TRACKS_ADDRS)


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
  def use_radar_tracks(self) -> bool:
    return bool(self.CP.flags & HyundaiFlags.CANFD) and Bus.radar in DBC[self.CP.carFingerprint] and not self.CP.radarUnavailable

  @property
  def use_radar_interface_ext(self) -> bool:
    return self.use_escc or self.use_radar_tracks or self.CP.flags & (HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC)

  def get_msg_src(self) -> str | None:
    if self.use_escc:
      return "ESCC"
    if self.CP.flags & (HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD_CAMERA_SCC):
      return "SCC_CONTROL" if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else "SCC11"

  def get_radar_ext_can_parser(self) -> CANParser:
    if self.use_radar_tracks:
      messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in RADAR_TRACKS_ADDRS]
      return CANParser(DBC[self.CP.carFingerprint][Bus.radar], messages, CanBus(self.CP).ACAN)

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
    if self.use_radar_tracks:
      return RADAR_TRACKS_ADDRS[-1]
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

    if self.use_radar_tracks:
      return self.update_radar_tracks(ret)

    for ii in range(1):
      msg_src = self.get_msg_src()
      msg = self.rcp.vl[msg_src]

      if ii not in self.pts:
        self.pts[ii] = structs.RadarData.RadarPoint()
        self.pts[ii].trackId = self.track_id
        self.track_id += 1

      valid = msg['ACC_ObjDist'] < 204.6 if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else msg['ACC_ObjStatus']
      if valid:
        self.pts[ii].dRel = msg['ACC_ObjDist']
        self.pts[ii].yRel = float('nan')  # FIXME-SP: Only some cars have lateral position from SCC
        self.pts[ii].vRel = msg['ACC_ObjRelSpd']

      else:
        del self.pts[ii]

    ret.points = list(self.pts.values())
    return ret

  def update_radar_tracks(self, ret: structs.RadarData) -> structs.RadarData:
    for addr in RADAR_TRACKS_ADDRS:
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      # 3 = measured, 4 = coasted
      if msg['STATE'] not in (3, 4):
        self.pts.pop(addr, None)
        continue

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      self.pts[addr].dRel = msg['LONG_DIST']
      self.pts[addr].yRel = msg['LAT_DIST']
      self.pts[addr].vRel = msg['REL_SPEED']

    ret.points = list(self.pts.values())
    return ret
