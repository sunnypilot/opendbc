#pragma once

//#include <stdint.h>
#include "opendbc/safety/board/can.h"

// UDS Service Types for sniffing
#define UDS_SERVICE_READ_DATA_BY_IDENTIFIER 0x22
#define UDS_SERVICE_WRITE_DATA_BY_IDENTIFIER 0x2E
#define UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL 0x10
#define UDS_SERVICE_ECU_RESET 0x11
#define UDS_SERVICE_SECURITY_ACCESS 0x27
#define UDS_SERVICE_TESTER_PRESENT 0x3E
#define UDS_SERVICE_ROUTINE_CONTROL 0x31
#define UDS_SERVICE_READ_DTC_INFORMATION 0x19
#define UDS_SERVICE_CLEAR_DIAGNOSTIC_INFORMATION 0x14

// UDS Response Types
#define UDS_RESPONSE_POSITIVE_OFFSET 0x40
#define UDS_RESPONSE_NEGATIVE 0x7F

// UDS Data Identifiers of interest
#define UDS_DID_VIN 0xF190
#define UDS_DID_ECU_SOFTWARE_NUMBER 0xF188
#define UDS_DID_ECU_SOFTWARE_VERSION 0xF189
#define UDS_DID_ECU_HARDWARE_NUMBER 0xF191
#define UDS_DID_ECU_SERIAL_NUMBER 0xF18C
#define UDS_DID_ACTIVE_DIAGNOSTIC_SESSION 0xF186

// ISO-TP Frame Types
#define ISOTP_SINGLE_FRAME 0x0
#define ISOTP_FIRST_FRAME 0x1
#define ISOTP_CONSECUTIVE_FRAME 0x2
#define ISOTP_FLOW_CONTROL_FRAME 0x3

// Maximum UDS data payload size
#define MAX_UDS_DATA_SIZE 256
#define MAX_UDS_SESSIONS 8

// UDS Session tracking
typedef struct {
  uint32_t tx_addr;
  uint32_t rx_addr;
  uint8_t bus;
  bool active;
  uint8_t sequence_number;
  uint16_t total_length;
  uint16_t received_length;
  uint8_t data[MAX_UDS_DATA_SIZE];
  uint32_t last_timestamp;
} uds_session_t;

// UDS message structure
typedef struct {
  uint8_t service_id;
  uint16_t data_identifier;  // For 0x22/0x2E services
  uint16_t data_length;
  uint8_t data[MAX_UDS_DATA_SIZE];
  bool is_response;
  bool is_negative_response;
  uint8_t negative_response_code;
  uint32_t timestamp;
} uds_message_t;

// UDS sniffer callbacks
typedef void (*uds_message_callback_t)(const uds_message_t *msg, uint32_t tx_addr, uint32_t rx_addr);

// Global UDS sniffer state
extern uds_session_t uds_sessions[MAX_UDS_SESSIONS];
extern uds_message_callback_t uds_callback;
extern bool uds_sniffer_enabled;

// UDS sniffer functions
void uds_sniffer_init(void);
void uds_sniffer_set_callback(uds_message_callback_t callback);
void uds_sniffer_enable(bool enable);
bool uds_sniffer_process_message(const CANPacket_t *msg);
void uds_sniffer_tick(void);

// Helper functions
bool is_uds_address(uint32_t addr);
bool is_isotp_frame(const CANPacket_t *msg);
uint8_t get_isotp_frame_type(const CANPacket_t *msg);
uint16_t get_isotp_data_length(const CANPacket_t *msg);

// UDS Data Identifier names (for debugging/logging)
const char* get_uds_did_name(uint16_t did);
const char* get_uds_service_name(uint8_t service_id);
const char* get_uds_nrc_name(uint8_t nrc);
