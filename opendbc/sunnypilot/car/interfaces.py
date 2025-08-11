"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.subaru.values import SubaruFlags
from opendbc.sunnypilot.car.hyundai.enable_radar_tracks import enable_radar_tracks as hyundai_enable_radar_tracks
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP, SubaruSafetyFlagsSP


def setup_interfaces(CP: structs.CarParams, CP_SP: structs.CarParamsSP, can_recv: CanRecvCallable = None, can_send: CanSendCallable = None) -> None:
  _initialize_radar_tracks(CP, CP_SP, can_recv, can_send)


def _initialize_radar_tracks(CP: structs.CarParams, CP_SP: structs.CarParamsSP, can_recv: CanRecvCallable = None, can_send: CanSendCallable = None) -> None:
  if CP.brand == 'hyundai':
    if CP.flags & HyundaiFlags.MANDO_RADAR and (CP.radarUnavailable or CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC):
      tracks_enabled = hyundai_enable_radar_tracks(can_recv, can_send, bus=0, addr=0x7d0)
      CP.radarUnavailable = not tracks_enabled


def _initialize_stop_and_go(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
  if CP.brand == 'subaru' and not CP.flags & (SubaruFlags.GLOBAL_GEN2 | SubaruFlags.HYBRID):
    stop_and_go = params.get_bool("SubaruStopAndGo")
    stop_and_go_manual_parking_brake = params.get_bool("SubaruStopAndGoManualParkingBrake")

    if stop_and_go:
      CP_SP.flags |= SubaruFlagsSP.STOP_AND_GO.value
    if stop_and_go_manual_parking_brake:
      CP_SP.flags |= SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE.value
    if stop_and_go or stop_and_go_manual_parking_brake:
      CP_SP.safetyParam |= SubaruSafetyFlagsSP.STOP_AND_GO
