"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import math
import numpy as np

from opendbc.car import DT_CTRL
from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.hyundai.values import CAR, HyundaiFlags

hyundai_checksum = mk_crc8_fun(CRC8J1850, init_crc=0xFD, xor_out=0xDF)

DAMPING_FF_SPEEDS = [20.0, 33.0]  # m/s
DAMPING_FF_GAINS = [0.0, 26.0]    # CAN torque counts / (deg/s)
DAMPING_FF_TAU = 0.06             # s
DAMPING_FF_MAX = 60.0             # CAN torque counts


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


class CameraSccCarController:
  def __init__(self, CP):
    self.camera_scc = bool(CP.flags & HyundaiFlags.CAMERA_SCC)
    self.ik2_damping_comp = CP.carFingerprint == CAR.GENESIS_G70_2024
    self.prev_desired_curvature = 0.0
    self.desired_steer_rate_filt = 0.0
    self._curv_to_steer_deg = math.degrees(CP.wheelbase * CP.steerRatio)

  def get_damping_feedforward(self, lat_active: bool, desired_curvature: float, v_ego: float) -> float:
    if not self.ik2_damping_comp:
      return 0.0
    if not lat_active:
      self.prev_desired_curvature = desired_curvature
      self.desired_steer_rate_filt = 0.0
      return 0.0

    dcurv_dt = (desired_curvature - self.prev_desired_curvature) / DT_CTRL
    self.prev_desired_curvature = desired_curvature
    desired_steer_rate = -dcurv_dt * self._curv_to_steer_deg
    alpha = DT_CTRL / (DAMPING_FF_TAU + DT_CTRL)
    self.desired_steer_rate_filt += alpha * (desired_steer_rate - self.desired_steer_rate_filt)
    gain = float(np.interp(v_ego, DAMPING_FF_SPEEDS, DAMPING_FF_GAINS))
    return float(np.clip(gain * self.desired_steer_rate_filt, -DAMPING_FF_MAX, DAMPING_FF_MAX))

  def update(self, CS):
    can_sends = []

    if not self.camera_scc:
      return can_sends

    can_sends.append(create_camera_scc_mdps12(self.packer, self.frame, self.CP, CS.mdps12))
    return can_sends
