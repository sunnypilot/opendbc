#pragma once

#include "opendbc/safety/uds_sniffer.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0])) // helper ? move somewhere else?
#define ECU_RESPONSE_ADDR(addr) ((addr) + 8)  // UDS response address is request address + 8
#define HKG_VERSION_REQUEST_LONG 0xF100

#define HKG_ADAS_UDS_ADDR 0x730u
#define HKG_RADAR_UDS_ADDR 0x7D0u
#define HKG_CAM_UDS_ADDR 0x7C4u

static const uint32_t HKG_UDS_REQUEST_ADDRS[] = {
  HKG_ADAS_UDS_ADDR,
  HKG_RADAR_UDS_ADDR,
  HKG_CAM_UDS_ADDR,
};

typedef struct {
  uint32_t ecu_address;
  char ecu_software_version[64];
  const AngleSteeringParams *steering_params;
} hkg_angle_fingerprint_t;


// Storage for captured UDS data
typedef struct {
  bool ecu_software_version_received;
  char ecu_software_version[64];
  uint32_t last_update_timestamp;
} hkg_uds_data_t;

// UDS callback function for Hyundai CANFD
static void hkg_canfd_uds_callback(const uds_message_t *msg, uint32_t tx_addr, uint32_t rx_addr);

// Helper functions
void hkg_canfd_init_uds_sniffer(void);
void hkg_canfd_disable_uds_sniffer(void);
static bool hkg_canfd_is_uds_addr(uint32_t addr);
static void hkg_canfd_process_software_version(uint32_t ecu_address, const uds_message_t *msg);
