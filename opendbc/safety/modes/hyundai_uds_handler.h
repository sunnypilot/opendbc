#pragma once

#include "opendbc/safety/modes/hyundai_uds_handler_declarations.h"

// Global storage for Hyundai UDS data
hyundai_uds_data_t hyundai_uds_data = {0};

void hyundai_canfd_uds_callback(const uds_message_t *msg, uint32_t tx_addr, uint32_t rx_addr) {
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
        case UDS_DID_VIN:
          hyundai_canfd_process_vin(msg);
          break;
        case UDS_DID_ECU_SOFTWARE_VERSION:
        case UDS_DID_ECU_SOFTWARE_NUMBER:
        case HYUNDAI_VERSION_REQUEST_LONG:
          hyundai_canfd_process_software_version(msg);
          break;
        case UDS_DID_ECU_HARDWARE_NUMBER:
          hyundai_canfd_process_hardware_version(msg);
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

  hyundai_uds_data.last_update_timestamp = msg->timestamp;
}

void hyundai_canfd_init_uds_sniffer(void) {
  // Clear stored data
  memset(&hyundai_uds_data, 0, sizeof(hyundai_uds_data));

  // Enable UDS sniffer
  uds_sniffer_enable(true);

  // Set our callback
  uds_sniffer_set_callback(hyundai_canfd_uds_callback);
}

bool hyundai_canfd_is_uds_addr(uint32_t addr) {
  // Check for Hyundai-specific UDS addresses
  return (addr == HYUNDAI_UDS_REQUEST_ADDR_ECU1 ||
          addr == HYUNDAI_UDS_RESPONSE_ADDR_ECU1 ||
          addr == HYUNDAI_UDS_REQUEST_ADDR_ECU2 ||
          addr == HYUNDAI_UDS_RESPONSE_ADDR_ECU2 ||
          addr == HYUNDAI_UDS_REQUEST_ADDR_ECU3 ||
          addr == HYUNDAI_UDS_RESPONSE_ADDR_ECU3);
}

void hyundai_canfd_process_vin(const uds_message_t *msg) {
  if (msg->data_length >= 17 && msg->data_length <= 17) {
    // VIN should be exactly 17 characters
    memcpy(hyundai_uds_data.vin, msg->data, 17);
    hyundai_uds_data.vin[17] = '\0';  // Null terminate
    hyundai_uds_data.vin_received = true;

    // Could log: "VIN captured: %s", hyundai_uds_data.vin
  }
}

void hyundai_canfd_process_software_version(const uds_message_t *msg) {
  if (msg->data_length > 0 && msg->data_length < sizeof(hyundai_uds_data.ecu_software_version)) {
    memcpy(hyundai_uds_data.ecu_software_version, msg->data, msg->data_length);
    hyundai_uds_data.ecu_software_version[msg->data_length] = '\0';  // Null terminate
    hyundai_uds_data.ecu_software_version_received = true;

    // Could log: "ECU Software Version: %s", hyundai_uds_data.ecu_software_version
  }
}

void hyundai_canfd_process_hardware_version(const uds_message_t *msg) {
  if (msg->data_length > 0 && msg->data_length < sizeof(hyundai_uds_data.ecu_hardware_version)) {
    memcpy(hyundai_uds_data.ecu_hardware_version, msg->data, msg->data_length);
    hyundai_uds_data.ecu_hardware_version[msg->data_length] = '\0';  // Null terminate
    hyundai_uds_data.ecu_hardware_version_received = true;

    // Could log: "ECU Hardware Version: %s", hyundai_uds_data.ecu_hardware_version
  }
}
