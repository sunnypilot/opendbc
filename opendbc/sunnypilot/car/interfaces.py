"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
# from opendbc.car.disable_ecu import disable_ecu
# from opendbc.car.hyundai.hyundaicanfd import CanBus as HyundaiCanBus
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.enable_radar_tracks import enable_radar_tracks as hyundai_enable_radar_tracks
from opendbc.sunnypilot.car.hyundai.values import RADAR_ADDRESS


def setup_interfaces(CP: structs.CarParams, CP_SP: structs.CarParamsSP, can_recv: CanRecvCallable = None, can_send: CanSendCallable = None) -> None:
  _initialize_radar_tracks(CP, can_recv, can_send)
  _disable_ecus(CP, CP_SP, can_recv, can_send)


def _initialize_radar_tracks(CP: structs.CarParams, can_recv: CanRecvCallable = None, can_send: CanSendCallable = None) -> None:
  if CP.brand == 'hyundai':
    if CP.flags & HyundaiFlags.MANDO_RADAR and CP.radarUnavailable:
      tracks_enabled = hyundai_enable_radar_tracks(can_recv, can_send, bus=0, addr=RADAR_ADDRESS)
      CP.radarUnavailable = not tracks_enabled

def _disable_ecus(CP: structs.CarParams, CP_SP: structs.CarParamsSP, can_recv: CanRecvCallable = None, can_send: CanSendCallable = None) -> None:
  assert CP
  assert CP_SP
  assert can_recv
  assert can_send
