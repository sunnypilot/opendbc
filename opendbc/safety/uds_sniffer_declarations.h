#pragma once

#include "opendbc/safety/board/can.h"

// UDS Service Types for sniffing
#define UDS_SERVICE_READ_DATA_BY_IDENTIFIER 0x22u
#define UDS_SERVICE_WRITE_DATA_BY_IDENTIFIER 0x2Eu
#define UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL 0x10u
#define UDS_SERVICE_SECURITY_ACCESS 0x27u

// UDS Response Types
#define UDS_RESPONSE_POSITIVE_OFFSET 0x40u
#define UDS_RESPONSE_NEGATIVE 0x7Fu

// UDS Data Identifiers of interest
#define UDS_DID_ECU_SOFTWARE_NUMBER 0xF188u
#define UDS_DID_ECU_SOFTWARE_VERSION 0xF189u
#define UDS_DID_ECU_SERIAL_NUMBER 0xF18Cu
#define UDS_DID_ACTIVE_DIAGNOSTIC_SESSION 0xF186u

// ISO-TP Frame Types
#define ISOTP_SINGLE_FRAME 0x0u
#define ISOTP_FIRST_FRAME 0x1u
#define ISOTP_CONSECUTIVE_FRAME 0x2u
#define ISOTP_FLOW_CONTROL_FRAME 0x3u

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
typedef bool (*is_uds_address_callback_t)(uint32_t addr);

// Global UDS sniffer state
extern uds_session_t uds_sessions[MAX_UDS_SESSIONS];
extern uds_message_callback_t uds_callback;
extern is_uds_address_callback_t is_uds_address_callback;
extern bool uds_sniffer_enabled;

// UDS sniffer functions
void uds_sniffer_init(void);
void uds_sniffer_set_callbacks(uds_message_callback_t callback, is_uds_address_callback_t callback2);
void uds_sniffer_enable(bool enable);
bool uds_sniffer_process_message(const CANPacket_t *msg);
void uds_sniffer_tick(void);

// Helper functions
static bool is_isotp_frame(const CANPacket_t *msg);
static uint8_t get_isotp_frame_type(const CANPacket_t *msg);
static uint16_t get_isotp_data_length(const CANPacket_t *msg);

extern void *memcpy(void *dest, const void *src, unsigned int len);
extern void *memset(void *str, int c, unsigned int n);
