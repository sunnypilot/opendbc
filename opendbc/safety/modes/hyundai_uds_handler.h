#pragma once

#include "opendbc/safety/modes/hyundai_uds_handler_declarations.h"

typedef struct {
  uint32_t ecu_address;
  hyundai_uds_data_t hyundai_uds_data;
} hyundai_uds_global_t;

static hyundai_uds_global_t hyundai_uds_global[] = {
  {HYUDAI_ADAS_UDS_ADDR, {0}},
  {HYUDAI_RADAR_UDS_ADDR, {0}},
  {HYUDAI_CAM_UDS_ADDR, {0}},
};

void hyundai_canfd_uds_callback(const uds_message_t *msg, uint32_t tx_addr, uint32_t rx_addr) {
  // Even though we only proces responses, I'm doing this to make it consistent.
  uint32_t ecu_address = msg->is_response ? rx_addr -8 : tx_addr;
  // Only process UDS responses (not requests)
  if (!msg->is_response && !msg->is_negative_response) {
    return;
  }

  // Only process messages from known Hyundai UDS addresses
  if (!hyundai_canfd_is_uds_addr(tx_addr) && !hyundai_canfd_is_uds_addr(rx_addr)) {
    return;
  }

  // Skip negative responses for now (could be logged for diagnostics)
  if (msg->is_negative_response) {
    // Could log: "UDS negative response: service=0x%02X, NRC=0x%02X"
    return;
  }

  // Process positive responses based on service type
  switch (msg->service_id) {
    case UDS_SERVICE_READ_DATA_BY_IDENTIFIER:
      switch (msg->data_identifier) {
        case UDS_DID_ECU_SOFTWARE_VERSION:
        case UDS_DID_ECU_SOFTWARE_NUMBER:
        case HYUNDAI_VERSION_REQUEST_LONG:
          hyundai_canfd_process_software_version(ecu_address, msg);
          break;
        default:
          // Other DIDs - could be logged for research
          break;
      }
      break;

    case UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL:
      // ECU session change detected
      break;

    case UDS_SERVICE_SECURITY_ACCESS:
      // Security access attempt detected
      break;

    default:
      // Other services - could be logged
      break;
  }

  // hyundai_uds_data.last_update_timestamp = msg->timestamp;
}

void hyundai_canfd_init_uds_sniffer(void) {
  // Clear stored data
  for (size_t i = 0; i < ARRAY_SIZE(hyundai_uds_global); i++) {
    memset(&hyundai_uds_global[i].hyundai_uds_data, 0, sizeof(hyundai_uds_global[i].hyundai_uds_data));
  }

  // Enable UDS sniffer
  uds_sniffer_enable(true);

  // Set our callback
  uds_sniffer_set_callback(hyundai_canfd_uds_callback);
}

void hyundai_canfd_disable_uds_sniffer(void) {
  uds_sniffer_enable(false);
  uds_sniffer_set_callback(NULL);
}

bool hyundai_canfd_is_uds_addr(uint32_t addr) {
  bool found = false;

  for (unsigned int i = 0; i < ARRAY_SIZE(HYUNDAI_UDS_REQUEST_ADDRS); i++) {
    uint32_t req = HYUNDAI_UDS_REQUEST_ADDRS[i];
    if (addr == req || addr == (req + 8u)) {
      found = true;
      break;  // still break to avoid extra iterations
    }
  }

  return found;
}

hyundai_uds_data_t *get_hyundai_uds_data_by_addr(uint32_t ecu_address) {
  for (size_t i = 0; i < ARRAY_SIZE(hyundai_uds_global); i++) {
    if (hyundai_uds_global[i].ecu_address == ecu_address) {
      return &hyundai_uds_global[i].hyundai_uds_data;
    }
  }
  return NULL;
}

void hyundai_canfd_process_software_version(uint32_t ecu_address, const uds_message_t *msg) {
  if (msg->data_length > 0) {
    hyundai_uds_data_t *ecu = get_hyundai_uds_data_by_addr(ecu_address);
    if (ecu != NULL && msg->data_length < sizeof(ecu->ecu_software_version) && !ecu->ecu_software_version_received) {
      memcpy(ecu->ecu_software_version, msg->data, msg->data_length);
      ecu->ecu_software_version[msg->data_length] = '\0';  // Null terminate
      ecu->ecu_software_version_received = true;
      ecu->last_update_timestamp = msg->timestamp;
    }
  }
}
