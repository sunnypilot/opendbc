import math

from opendbc.can.parser import CANParser

from opendbc.car import structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC

from opendbc.sunnypilot.car.hyundai.escc import EnhancedSmartCruiseControl

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP):
  if DBC[CP.carFingerprint]['radar'] is None:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint]['radar'], messages, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)
    self.ESCC = EnhancedSmartCruiseControl(CP)

    # Override radar parser with the ESCC parser and trigger message if ESCC is enabled
    if self.ESCC.enabled:
      self.rcp = self.ESCC.get_radar_parser_escc()
      self.trigger_msg = self.ESCC.trigger_msg

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update_strings(can_strings)
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

    errors = []

    if not self.rcp.can_valid:
      errors.append("canError")
    ret.errors = errors

    if self.ESCC.enabled:
      self._update_escc()
    else:
      self._update_radar_tracks()

    ret.points = list(self.pts.values())
    return ret

  def _update_radar_tracks(self):
    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
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

  def _update_escc(self):
    msg_src = "ESCC"
    msg = self.rcp.vl[msg_src]

    valid = msg['ACC_ObjStatus']
    for ii in range(1):
      if valid:
        if ii not in self.pts:
          self.pts[ii] = structs.RadarData.RadarPoint.new_message()
          self.pts[ii].trackId = self.track_id
          self.track_id += 1
        self.pts[ii].measured = True
        self.pts[ii].dRel = msg['ACC_ObjDist']
        self.pts[ii].yRel = -msg['ACC_ObjLatPos'] if self.ESCC.enabled else float('nan')
        self.pts[ii].vRel = msg['ACC_ObjRelSpd']
        self.pts[ii].aRel = float('nan')
        self.pts[ii].yvRel = float('nan')

      else:
        if ii in self.pts:
          del self.pts[ii]
