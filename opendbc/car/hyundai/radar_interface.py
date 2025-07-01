from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt

MANDO_RADAR_START_ADDR = 0x500
MANDO_RADAR_MSG_COUNT = 32

MRREVO14F_RADAR_START_ADDR = 0x602
MRREVO14F_RADAR_MSG_COUNT = 16
# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/

def get_radar_can_parser(CP, radar_start_addr, radar_msg_count):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(radar_start_addr, radar_start_addr + radar_msg_count)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.CP_flags = CP.flags
    if self.CP_flags & HyundaiFlags.MRREVO14F_RADAR:
      self.radar_start_addr = MRREVO14F_RADAR_START_ADDR
      self.radar_msg_count = MRREVO14F_RADAR_MSG_COUNT
    else:
      self.radar_start_addr = MANDO_RADAR_START_ADDR
      self.radar_msg_count = MANDO_RADAR_MSG_COUNT
    self.updated_messages = set()
    self.trigger_msg = self.radar_start_addr + self.radar_msg_count - 1
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP, self.radar_start_addr, self.radar_msg_count)

    if self.rcp is None:
      self.initialize_radar_ext(self.trigger_msg)

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

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.use_radar_interface_ext:
      return self.update_ext(ret)

    return ret
