import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

MANDO_RADAR_ADDR = 0x500
MANDO_RADAR_COUNT = 32
MRREVO14F_RADAR_ADDR = 0x602
MRREVO14F_RADAR_COUNT = 16
MRR30_RADAR_ADDR = 0x210
MRR30_RADAR_COUNT = 16
MRR35_RADAR_ADDR = 0x3A5
MRR35_RADAR_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP, radar_addr, radar_count):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(radar_addr, radar_addr + radar_count)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.CP_flags = CP.flags
    if self.CP_flags & HyundaiFlags.MRREVO14F_RADAR:
      self.radar_addr, self.radar_count = MRREVO14F_RADAR_ADDR, MRREVO14F_RADAR_COUNT
    elif self.CP_flags & HyundaiFlags.MRR30_RADAR:
      self.radar_addr, self.radar_count = MRR30_RADAR_ADDR, MRR30_RADAR_COUNT
    elif self.CP_flags & HyundaiFlags.MRR35_RADAR:
      self.radar_addr, self.radar_count = MRR35_RADAR_ADDR, MRR35_RADAR_COUNT
    else:
      self.radar_addr, self.radar_count = MANDO_RADAR_ADDR, MANDO_RADAR_COUNT
    self.updated_messages = set()
    self.trigger_msg = self.radar_addr + self.radar_count - 1
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP, self.radar_addr, self.radar_count)

    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.CP_SP.flags & HyundaiFlagsSP.RADAR_LEAD_ONLY:
      if self.use_radar_interface_ext:
        return self.update_ext(ret)

    for addr in range(self.radar_addr, self.radar_addr + self.radar_count):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if self.CP_SP.flags & HyundaiFlagsSP.RADAR_FULL_RADAR:

        if self.CP_flags & HyundaiFlags.MRREVO14F_RADAR:
          msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
          for i in ("1", "2"):
            track_key = f"{addr}_{i}"
            dist = msg[f"{i}_DISTANCE"]
            if track_key not in self.pts:
              self.pts[track_key] = structs.RadarData.RadarPoint()
              self.pts[track_key].trackId = self.track_id
              self.track_id += 1
            if dist != 255.75:
              pt = self.pts[track_key]
              pt.measured = True
              pt.dRel = dist
              pt.yRel = msg[f"{i}_LATERAL"]
              pt.vRel = msg[f"{i}_SPEED"]
              pt.aRel = float('nan')
              pt.yvRel = float('nan')
            else:
              del self.pts[track_key]

        elif self.CP_flags & HyundaiFlags.MRR30_RADAR:
          msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
          for i in ("1", "2"):
            track_key = f"{addr}_{i}"
            if track_key not in self.pts:
              self.pts[track_key] = structs.RadarData.RadarPoint()
              self.pts[track_key].trackId = self.track_id
              self.track_id += 1
            if msg[f"{i}_STATE"] in (3, 4):
              pt = self.pts[track_key]
              pt.measured = True
              pt.dRel = msg[f"{i}_LONG_DIST"]
              pt.yRel = msg[f"{i}_LAT_DIST"]
              pt.vRel = msg[f"{i}_REL_SPEED"]
              pt.aRel = float('nan')  # confirm REL_ACCEL factor
              pt.yvRel = float('nan')
            else:
              del self.pts[track_key]

        elif self.CP_flags & HyundaiFlags.MRR35_RADAR:
          msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
          if addr not in self.pts:
            self.pts[addr] = structs.RadarData.RadarPoint()
            self.pts[addr].trackId = self.track_id
            self.track_id += 1
          if msg['STATE'] in (3, 4):
            self.pts[addr].measured = True
            self.pts[addr].dRel = msg['LONG_DIST']
            self.pts[addr].yRel = msg['LAT_DIST']
            self.pts[addr].vRel = msg['REL_SPEED']
            self.pts[addr].aRel = msg['REL_ACCEL']  # confirm factor
            self.pts[addr].yvRel = float('nan')
          else:
            del self.pts[addr]

        else:
          msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
          if addr not in self.pts:
            self.pts[addr] = structs.RadarData.RadarPoint()
            self.pts[addr].trackId = self.track_id
            self.track_id += 1
          if msg['STATE'] in (3, 4):
            azimuth = math.radians(msg['AZIMUTH'])
            self.pts[addr].measured = True
            self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
            self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
            self.pts[addr].vRel = msg['REL_SPEED']
            self.pts[addr].aRel = msg['REL_ACCEL']
            self.pts[addr].yvRel = float('nan')
          else:
            del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret
