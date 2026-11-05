"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


def create_ccnc_mdps(packer, CAN, mdps):
  # report LKA torque inactive/unavailable so the camera LFA stays in standby
  return packer.make_can_msg("MDPS", CAN.CAM, {**mdps, "MDPS_LkaToiActvSta": 0, "MDPS_LkaToiUnblSta": 1})


def create_ccnc(packer, CAN, msg_161, msg_162, lat_enabled):
  msg_161.update({
    "LKA_ICON": 4 if lat_enabled else 3,        # green lateral enabled, gray otherwise
    "LFA_ICON": 2 if lat_enabled else 1,        # green lateral enabled, gray otherwise
    "LANELINE_LEFT": 6 if lat_enabled else 0,   # green lateral enabled, gray otherwise
    "LANELINE_RIGHT": 6 if lat_enabled else 0,  # green lateral enabled, gray otherwise
    "CENTERLINE": 1 if lat_enabled else 0,      # green lateral enabled, hidden otherwise (no gray)
    "DAW_ICON": 0,
  })

  return [packer.make_can_msg(msg, CAN.ECAN, data) for msg, data in [("CCNC_0x161", msg_161), ("CCNC_0x162", msg_162)]]


class CcncCarController:
  def __init__(self, CP, CP_SP):
    self.ccnc_non_hda2 = bool(CP_SP.flags & HyundaiFlagsSP.CCNC and not (CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG))

  def update(self, CS, CC, CC_SP):
    if not self.ccnc_non_hda2:
      return []

    can_sends = [create_ccnc_mdps(self.packer, self.CAN, CS.mdps)]
    if self.frame % 5 == 0:
      lat_enabled = CC_SP.mads.enabled if CC_SP.mads.available else CC.enabled
      can_sends.extend(create_ccnc(self.packer, self.CAN, CS.msg_161, CS.msg_162, lat_enabled))
    return can_sends
