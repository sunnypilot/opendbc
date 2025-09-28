#pragma once

#include "opendbc/safety/modes/hyundai_uds_handler_declarations.h"

typedef struct {
  uint32_t ecu_address;
  hkg_uds_data_t hkg_uds_data;
} hkg_uds_global_t;

static hkg_uds_global_t hkg_uds_global[] = {
  {HKG_ADAS_UDS_ADDR, {0}},
  {HKG_RADAR_UDS_ADDR, {0}},
  {HKG_CAM_UDS_ADDR, {0}},
};

static void hkg_canfd_uds_callback(const uds_message_t *msg, uint32_t tx_addr, uint32_t rx_addr) {
  // Even though we only process responses, I'm doing this to make it consistent.
  uint32_t ecu_address = msg->is_response ? rx_addr -8 : tx_addr;
  // Only process UDS responses (not requests)
  if ((msg->is_response && !msg->is_negative_response) && (hkg_canfd_is_uds_addr(tx_addr) || hkg_canfd_is_uds_addr(rx_addr))) {
    // Skip negative responses for now (could be logged for diagnostics)

    // Process positive responses based on service type
    switch (msg->service_id) {
      case UDS_SERVICE_READ_DATA_BY_IDENTIFIER:
        switch (msg->data_identifier) {
          case UDS_DID_ECU_SOFTWARE_VERSION:
          case UDS_DID_ECU_SOFTWARE_NUMBER:
          case HKG_VERSION_REQUEST_LONG:
            hkg_canfd_process_software_version(ecu_address, msg);
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
  }
}

static bool hkg_canfd_is_uds_addr(uint32_t addr) {
  bool found = false;

  for (unsigned int i = 0; i < ARRAY_SIZE(HKG_UDS_REQUEST_ADDRS); i++) {
    uint32_t req = HKG_UDS_REQUEST_ADDRS[i];
    if (addr == req || addr == (req + 8u)) {
      found = true;
      break;  // still break to avoid extra iterations
    }
  }

  return found;
}

void hkg_canfd_init_uds_sniffer(void) {
  // Clear stored data
  for (size_t i = 0; i < ARRAY_SIZE(hkg_uds_global); i++) {
    memset(&hkg_uds_global[i].hkg_uds_data, 0, sizeof(hkg_uds_global[i].hkg_uds_data));
  }

  // Enable UDS sniffer
  uds_sniffer_enable(true);

  // Set our callback
  uds_sniffer_set_callbacks(hkg_canfd_uds_callback, hkg_canfd_is_uds_addr);
}

void hkg_canfd_disable_uds_sniffer(void) {
  uds_sniffer_enable(false);
  uds_sniffer_set_callbacks(NULL, NULL);
}

hkg_uds_data_t *get_hkg_uds_data_by_addr(uint32_t ecu_address) {
  for (size_t i = 0; i < ARRAY_SIZE(hkg_uds_global); i++) {
    if (hkg_uds_global[i].ecu_address == ecu_address) {
      return &hkg_uds_global[i].hkg_uds_data;
    }
  }
  return NULL;
}

static void hkg_canfd_process_software_version(uint32_t ecu_address, const uds_message_t *msg) {
  if (msg->data_length > 0) {
    hkg_uds_data_t *ecu = get_hkg_uds_data_by_addr(ecu_address);
    if (ecu != NULL && msg->data_length < sizeof(ecu->ecu_software_version) && !ecu->ecu_software_version_received) {
      memcpy(ecu->ecu_software_version, msg->data, msg->data_length);
      ecu->ecu_software_version[msg->data_length] = '\0';  // Null terminate
      ecu->ecu_software_version_received = true;
      ecu->last_update_timestamp = msg->timestamp;
    }
  }
}
