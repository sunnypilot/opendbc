#pragma once

#include "opendbc/safety/safety_declarations.h"
#include "opendbc/safety/modes/hyundai_canfd.h"

static void hyundai_canfd_adas_interceptor_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  const int pt_bus = hyundai_canfd_lka_steering ? 1 : 0;
  const int scc_bus = hyundai_camera_scc ? 2 : pt_bus;

  
}

static bool hyundai_canfd_adas_interceptor_tx_hook(const CANPacket_t *to_send) {
  bool tx = false;
  return tx;
}

static safety_config hyundai_canfd_adas_interceptor_init(uint16_t param) {
  safety_config ret = nullptr;

  return ret;
}

const safety_hooks hyundai_canfd_adas_interceptor_hooks = {
  .init = hyundai_canfd_init,
  .rx = hyundai_canfd_rx_hook,
  .tx = hyundai_canfd_tx_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
