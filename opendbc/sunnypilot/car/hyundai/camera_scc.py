"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.hyundai.values import HyundaiFlags

hyundai_checksum = mk_crc8_fun(CRC8J1850, init_crc=0xFD, xor_out=0xDF)

DAW_TAKE_A_BREAK = 6


def create_camera_scc_mdps12(packer, frame, CP, mdps12):
  values = mdps12.copy()

  values.update({
    "CF_Mdps_ToiActive": 0,
    "CF_Mdps_ToiUnavail": 1,
    "CF_Mdps_MsgCount2": frame % 0x100,
    "CF_Mdps_Chksum2": 0,
  })

  dat = packer.make_can_msg("MDPS12", 2, values)[1]
  payload = dat[:3] + dat[4:]
  if CP.flags & HyundaiFlags.CHECKSUM_CRC8:
    values["CF_Mdps_Chksum2"] = hyundai_checksum(payload)
  else:
    values["CF_Mdps_Chksum2"] = sum(payload) % 256

  return packer.make_can_msg("MDPS12", 2, values)


def create_camera_scc_lkas12(packer, lkas12, daw_status):
  values = lkas12.copy()

  # suppress the camera's driver attention "take a break" popup, the driver is monitored by sunnypilot
  if values["CF_LkasDawStatus"] == DAW_TAKE_A_BREAK:
    values.update({
      "CF_LkasDawStatus": daw_status,
      "CF_Lkas12_Unknown4": 0,
    })

  return packer.make_can_msg("LKAS12", 0, values)


class CameraSccCarController:
  def __init__(self, CP):
    self.camera_scc = bool(CP.flags & HyundaiFlags.CAMERA_SCC)
    self.lkas12_daw_status = 0

  def update(self, CS):
    can_sends = []

    if not self.camera_scc:
      return can_sends

    can_sends.append(create_camera_scc_mdps12(self.packer, self.frame, self.CP, CS.mdps12))

    if self.frame % 10 == 0 and CS.lkas12:
      if CS.lkas12["CF_LkasDawStatus"] != DAW_TAKE_A_BREAK:
        self.lkas12_daw_status = CS.lkas12["CF_LkasDawStatus"]
      can_sends.append(create_camera_scc_lkas12(self.packer, CS.lkas12, self.lkas12_daw_status))
    return can_sends
