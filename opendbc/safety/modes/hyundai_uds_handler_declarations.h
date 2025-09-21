#pragma once

#include "opendbc/safety/uds_sniffer.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0])) // helper ? move somewhere else?
#define ECU_RESPONSE_ADDR(addr) ((addr) + 8)  // UDS response address is request address + 8
#define HYUNDAI_VERSION_REQUEST_LONG 0xF100

#define HYUDAI_ADAS_UDS_ADDR 0x730u
#define HYUDAI_RADAR_UDS_ADDR 0x7D0u
#define HYUDAI_CAM_UDS_ADDR 0x7C4u

static const uint32_t HYUNDAI_UDS_REQUEST_ADDRS[] = {
  HYUDAI_ADAS_UDS_ADDR,
  HYUDAI_RADAR_UDS_ADDR,
  HYUDAI_CAM_UDS_ADDR,
};

typedef struct {
  uint32_t ecu_address;
  char ecu_software_version[64];
  const AngleSteeringParams *steering_params;
} hyundai_angle_fingerprint_t;


// Storage for captured UDS data
typedef struct {
  bool ecu_software_version_received;
  char ecu_software_version[64];
  uint32_t last_update_timestamp;
} hyundai_uds_data_t;

// UDS callback function for Hyundai CANFD
void hyundai_canfd_uds_callback(const uds_message_t *msg, uint32_t tx_addr, uint32_t rx_addr);

// Helper functions
void hyundai_canfd_init_uds_sniffer(void);
void hyundai_canfd_disable_uds_sniffer(void);
bool hyundai_canfd_is_uds_addr(uint32_t addr);
void hyundai_canfd_process_software_version(uint32_t ecu_address, const uds_message_t *msg);
