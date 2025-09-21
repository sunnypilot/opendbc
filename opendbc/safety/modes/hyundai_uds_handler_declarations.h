#pragma once

#include "opendbc/safety/uds_sniffer.h"

// Hyundai-specific UDS addresses
#define HYUNDAI_UDS_REQUEST_ADDR_ECU1 0x730   // ADAS ECU
#define HYUNDAI_UDS_RESPONSE_ADDR_ECU1 0x738
#define HYUNDAI_UDS_REQUEST_ADDR_ECU2 0x7D0   // Radar ECU
#define HYUNDAI_UDS_RESPONSE_ADDR_ECU2 0x7D8
#define HYUNDAI_UDS_REQUEST_ADDR_ECU3 0x7C4   // CAM
#define HYUNDAI_UDS_RESPONSE_ADDR_ECU3 0x7CC

// Storage for captured UDS data
typedef struct {
  bool vin_received;
  char vin[18];  // VIN is 17 chars + null terminator
  bool ecu_software_version_received;
  char ecu_software_version[64];
  bool ecu_hardware_version_received;
  char ecu_hardware_version[64];
  uint32_t last_update_timestamp;
} hyundai_uds_data_t;

extern hyundai_uds_data_t hyundai_uds_data;

// UDS callback function for Hyundai CANFD
void hyundai_canfd_uds_callback(const uds_message_t *msg, uint32_t tx_addr, uint32_t rx_addr);

// Helper functions
void hyundai_canfd_init_uds_sniffer(void);
bool hyundai_canfd_is_uds_addr(uint32_t addr);
void hyundai_canfd_process_vin(const uds_message_t *msg);
void hyundai_canfd_process_software_version(const uds_message_t *msg);
void hyundai_canfd_process_hardware_version(const uds_message_t *msg);
