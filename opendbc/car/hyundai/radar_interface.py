import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags
from opendbc.car.carlog import carlog

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP):
  if CP.flags & HyundaiFlags.CANFD_RADAR:
    messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range( 0x180, 0x185)]
    return CANParser(DBC[CP.carFingerprint][Bus.pt], messages, 1)
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.updated_messages = set()
    if CP.flags & HyundaiFlags.CANFD_RADAR:
      self.trigger_msg = 0x180
    else:
      self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

    if self.rcp is None:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    try:
      if self.CP.flags & HyundaiFlags.CANFD_RADAR:
        rr = self._update_canfd(self.updated_messages)
      else:
        rr = self._update(self.updated_messages)
    except Exception as e:
      self.updated_messages.clear()
      carlog.error(f"An unexpected error occurred: {e}")
      return None

    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.use_radar_interface_ext:
      return self.update_ext(ret)

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

    ret.points = list(self.pts.values())
    return ret

  def _update_canfd(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    # pts keeps latest radar status
    self.pts.clear()

    for addr in range( 0x180, 0x185):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
      track_id1 = int(msg['TRACK_ID1'])
      track_heartbeat1 = int(msg['TRACK_HEARTBEAT1'])

      if track_id1 != 0 and track_heartbeat1 != 0:
        self.pts[track_id1] = structs.RadarData.RadarPoint()
        self.pts[track_id1].trackId = track_id1

        self.pts[track_id1].dRel = float(msg['LONG_DIST1'])
        self.pts[track_id1].yRel = float(msg['LAT_DIST1'])
        self.pts[track_id1].vRel = float(msg['REL_SPEED1'])
        self.pts[track_id1].yvRel = float(msg['LAT_SPEED1'])
        self.pts[track_id1].aRel = float(msg['REL_ACCEL1'])
        self.pts[track_id1].measured = True

      track_id2 = int(msg['TRACK_ID2'])
      track_heartbeat2 = int(msg['TRACK_HEARTBEAT2'])

      if track_id2 != 0 and track_heartbeat2 != 0:
        self.pts[track_id2] = structs.RadarData.RadarPoint()
        self.pts[track_id2].trackId = track_id2

        self.pts[track_id2].dRel = float(msg['LONG_DIST2'])
        self.pts[track_id2].yRel = float(msg['LAT_DIST2'])
        self.pts[track_id2].vRel = float(msg['REL_SPEED2'])
        self.pts[track_id2].yvRel = float(msg['LAT_SPEED2'])
        self.pts[track_id2].aRel = float(msg['REL_ACCEL2'])
        self.pts[track_id2].measured = True

    ret.points = list(self.pts.values())
    return ret
