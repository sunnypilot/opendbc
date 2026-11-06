"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""


from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


def create_ccnc_mdps(packer, CAN, mdps):
  values = mdps.copy()

  values.update({
    "MDPS_LkaToiActvSta": 0,
    "MDPS_LkaToiUnblSta": 1,
  })

  return packer.make_can_msg("MDPS", CAN.CAM, values)


def create_ccnc(packer, CAN, msg_161, msg_162, enabled, mads):
  lat_active = mads.lat_active if mads.enable_mads else enabled
  paused = mads.enable_mads and mads.paused

  values_161 = msg_161.copy()
  values_161.update({
    "LKA_ICON": 4 if lat_active else 3,
    "LFA_ICON": 2 if lat_active else 1 if paused else 0,
    "LANELINE_LEFT": 6 if lat_active else 0,
    "LANELINE_RIGHT": 6 if lat_active else 0,
    "CENTERLINE": 1 if lat_active else 0,
    "DAW_ICON": 0,
  })

  values_162 = msg_162.copy()

  return [
    packer.make_can_msg("CCNC_0x161", CAN.ECAN, values_161),
    packer.make_can_msg("CCNC_0x162", CAN.ECAN, values_162),
  ]


class CcncCarController:
  def __init__(self, CP, CP_SP):
    self.ccnc_lfa_steering = bool(CP_SP.flags & HyundaiFlagsSP.CCNC and not (CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG))

  def update(self, CS, CC):
    can_sends = []

    if not self.ccnc_lfa_steering:
      return can_sends

    can_sends.append(create_ccnc_mdps(self.packer, self.CAN, CS.mdps))
    if self.frame % 5 == 0:
      can_sends.extend(create_ccnc(self.packer, self.CAN, CS.msg_161, CS.msg_162, CC.enabled, self.mads))
    return can_sends
