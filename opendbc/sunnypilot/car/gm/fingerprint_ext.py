import time

from opendbc.car.can_definitions import CanRecvCallable
from opendbc.car.gm.values import CAR as GM


def _classify_bolt_by_ascm_status(can_recv: CanRecvCallable) -> str:
  # On some non-ACC Bolt EUV trims, 0x370 is present but always all-zero.
  # ACC variants send non-zero fields (e.g. ACCGapLevel), so use this to split.
  saw_ascm_status = False
  saw_bus2_traffic = False
  frames = 0
  max_frames = 40
  deadline = time.monotonic() + 1.0

  while frames < max_frames and time.monotonic() < deadline:
    can_packets = can_recv(wait_for_one=False)
    if len(can_packets) == 0:
      time.sleep(0.01)
      continue

    for can_packet in can_packets:
      for can in can_packet:
        if can.src == 2:
          saw_bus2_traffic = True
        if can.src == 2 and can.address == 0x370:
          saw_ascm_status = True
          if any(can.dat):
            return "acc"

    frames += len(can_packets)

  if saw_ascm_status:
    return "non_acc_2nd_gen"
  if saw_bus2_traffic:
    return "non_acc_1st_gen"
  return "acc"


def remap_candidate(candidate: str | None, can_recv: CanRecvCallable) -> str | None:
  if candidate != GM.CHEVROLET_BOLT_EUV:
    return candidate

  bolt_variant = _classify_bolt_by_ascm_status(can_recv)
  if bolt_variant == "non_acc_2nd_gen":
    return GM.CHEVROLET_BOLT_NON_ACC_2ND_GEN
  if bolt_variant == "non_acc_1st_gen":
    return GM.CHEVROLET_BOLT_NON_ACC_1ST_GEN
  return candidate
