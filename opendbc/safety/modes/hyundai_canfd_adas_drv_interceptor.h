#pragma once

#include "opendbc/safety/safety_declarations.h"
#include "opendbc/safety/modes/hyundai_canfd.h"

#define HYUNDAI_CANFD_ADAS_DRV_SCC_MSGS(bus) \
{0x1A0, bus, 8, .check_relay = false},  /* CRUISE_BUTTON */   \

#define ADAS_DRV_BUS 2
#define CAR_BUS 0
#define ESCC_MASK 0x800

uint32_t sunnypilot_detected_last = 0;
bool block_adas_drv_ecu = false;

static const CanMsg HYUNDAI_CANFD_ADAS_DRV_TX_MSGS[] = {
  HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
};

static void hyundai_canfd_adas_drv_interceptor_rx_hook(const CANPacket_t *to_push) {
 
}

static bool hyundai_canfd_adas_drv_interceptor_tamper_hook(CANPacket_t * to_send) {
  int bus = GET_BUS(to_send);
  int addr = GET_ADDR(to_send);

  const int is_scc_msg = addr == 0x1A0;
  if (is_scc_msg && bus == CAR_BUS) {
    sunnypilot_detected_last = MICROSECOND_TIMER->CNT;
  }

  if (block_adas_drv_ecu && is_scc_msg && bus == ADAS_DRV_BUS) {
    to_send->addr = 0x1A0 | ESCC_MASK; // Change SCC_CONTROL address to avoid being processed by the car
  }

  return true;
}

// static safety_config hyundai_canfd_adas_interceptor_init(uint16_t param) {
//   safety_config ret = nullptr;
//
//   return ret;
// }


static bool hyundai_canfd_adas_drv_interceptor_fwd_hook(int bus_num, int addr) {
  const int is_scc_msg = addr == 0x1A0;
  const uint32_t ts = MICROSECOND_TIMER->CNT;
  
  // Update the last detected timestamp if an SCC message is from CAR_BUS
  if (bus_num == CAR_BUS && is_scc_msg) {
    sunnypilot_detected_last = ts;
  }

  // Default forwarding logic
  int bus_dst = (bus_num == CAR_BUS) ? ADAS_DRV_BUS : CAR_BUS;

  // Update the scc_block_allowed status based on elapsed time
  const uint32_t ts_elapsed = get_ts_elapsed(ts, sunnypilot_detected_last);
  block_adas_drv_ecu = (ts_elapsed <= 150000);

  // If we are allowed to block, and this is an scc msg coming from ADAS (or somehow we are sending it TO the ADAS) we block
  return !(block_adas_drv_ecu && is_scc_msg && (bus_num == ADAS_DRV_BUS || bus_dst == ADAS_DRV_BUS));
}

const safety_hooks hyundai_canfd_adas_drv_interceptor_hooks = {
  // .init = hyundai_canfd_init,
  .rx = hyundai_canfd_adas_drv_interceptor_rx_hook,
  .tamper = hyundai_canfd_adas_drv_interceptor_tamper_hook,
  .fwd = hyundai_canfd_adas_drv_interceptor_fwd_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
