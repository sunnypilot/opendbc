#pragma once

#include "opendbc/safety/board/can.h"
#include "opendbc/safety/uds_sniffer.h"

#include "opendbc/safety/uds_sniffer_declarations.h"
#include <stddef.h>


// Global UDS sniffer state
uds_session_t uds_sessions[MAX_UDS_SESSIONS];
uds_message_callback_t uds_callback = NULL;
is_uds_address_callback_t is_uds_address_callback = NULL;
bool uds_sniffer_enabled = false;

void uds_sniffer_init(void) {
  // Initialize all sessions as inactive
  for (int i = 0; i < MAX_UDS_SESSIONS; i++) {
    uds_sessions[i].active = false;
    uds_sessions[i].tx_addr = 0;
    uds_sessions[i].rx_addr = 0;
    uds_sessions[i].bus = 0;
    uds_sessions[i].sequence_number = 0;
    uds_sessions[i].total_length = 0;
    uds_sessions[i].received_length = 0;
    uds_sessions[i].last_timestamp = 0;
  }
  uds_callback = NULL;
  is_uds_address_callback = NULL;
  uds_sniffer_enabled = false;
}

void uds_sniffer_set_callbacks(uds_message_callback_t callback, is_uds_address_callback_t callback2) {
  uds_callback = callback;
  is_uds_address_callback = callback2;
}

void uds_sniffer_enable(bool enable) {
  uds_sniffer_enabled = enable;
}

static bool is_isotp_frame(const CANPacket_t *msg) {
  bool result = false;
  if (GET_LEN(msg) != 0u) {
    uint8_t pci = msg->data[0] >> 4;
    result = (pci <= ISOTP_FLOW_CONTROL_FRAME);
  }
  return result;
}

static uint8_t get_isotp_frame_type(const CANPacket_t *msg) {
  return (msg->data[0] >> 4) & 0x0Fu;
}

static uint16_t get_isotp_data_length(const CANPacket_t *msg) {
  uint8_t frame_type = get_isotp_frame_type(msg);
  uint16_t result = 0;

  if (frame_type == ISOTP_SINGLE_FRAME) {
    result = msg->data[0] & 0x0Fu;
  } else if (frame_type == ISOTP_FIRST_FRAME) {
    result = ((msg->data[0] & 0x0Fu) << 8) | msg->data[1];
  } else {
    result = 0;
  }

  return result;
}

static uds_session_t* find_or_create_session(uint32_t tx_addr, uint32_t rx_addr, uint8_t bus) {
  uds_session_t* free_session = NULL;
  uint32_t oldest_timestamp = UINT32_MAX;
  uds_session_t* oldest_session = NULL;
  uds_session_t* session = NULL;

  // First, try to find existing session
  for (int i = 0; i < MAX_UDS_SESSIONS; i++) {
    if (uds_sessions[i].active &&
        (uds_sessions[i].tx_addr == tx_addr) &&
        (uds_sessions[i].rx_addr == rx_addr) &&
        (uds_sessions[i].bus == bus)) {
      session = &uds_sessions[i];
    }

    if ((!uds_sessions[i].active) && (free_session == NULL)) {
      free_session = &uds_sessions[i];
    }

    if (uds_sessions[i].last_timestamp < oldest_timestamp) {
      oldest_timestamp = uds_sessions[i].last_timestamp;
      oldest_session = &uds_sessions[i];
    }
  }

  // Use free session if available, otherwise reuse oldest
  if (session == NULL)
  {
    session = free_session ? free_session : oldest_session;

    if (session != NULL) {
      session->active = true;
      session->tx_addr = tx_addr;
      session->rx_addr = rx_addr;
      session->bus = bus;
      session->sequence_number = 0u;
      session->total_length = 0u;
      session->received_length = 0u;
      session->last_timestamp = microsecond_timer_get();
    }
  }

  return session;
}

static void parse_and_callback_uds_message(uds_session_t* session) {
  if (session && (session->received_length >= 1u) && uds_callback) {
    uds_message_t msg = {0};
    msg.timestamp = session->last_timestamp;

    const uint8_t *data = session->data;
    char service_id = data[0];

    // Check if it's a negative response
    if (service_id == UDS_RESPONSE_NEGATIVE) {
      if (session->received_length >= 3u) {
        msg.is_negative_response = true;
        msg.service_id = data[1]; // Original service ID
        msg.negative_response_code = data[2];
        msg.data_length = session->received_length - 3u;
        if (msg.data_length > 0u) {
          (void)memcpy(msg.data, &data[3], MIN(msg.data_length, MAX_UDS_DATA_SIZE));
        }
      }
    } else {
      // Check if it's a positive response
      if (service_id >= UDS_RESPONSE_POSITIVE_OFFSET) {
        msg.is_response = true;
        msg.service_id = service_id - UDS_RESPONSE_POSITIVE_OFFSET;
      } else {
        msg.service_id = service_id;
      }

      // Parse data identifier for read/write data services
      if (((msg.service_id == UDS_SERVICE_READ_DATA_BY_IDENTIFIER) ||
           (msg.service_id == UDS_SERVICE_WRITE_DATA_BY_IDENTIFIER)) &&
          (session->received_length >= 3u)) {
        msg.data_identifier = (data[1] << 8) | data[2];
        msg.data_length = (uint8_t) (session->received_length - 3u);
        if (msg.data_length > 0u) {
          (void)memcpy(msg.data, &data[3], MIN(msg.data_length, MAX_UDS_DATA_SIZE));
        }
      } else {
        msg.data_length = session->received_length - 1u;
        if (msg.data_length > 0u) {
          (void)memcpy(msg.data, &data[1], MIN(msg.data_length, MAX_UDS_DATA_SIZE));
        }
      }
    }

    uds_callback(&msg, session->tx_addr, session->rx_addr);
  }
}

void uds_sniffer_process_message(const CANPacket_t *msg) {
  if (uds_sniffer_enabled && is_uds_address_callback(msg->addr) && is_isotp_frame(msg)) {
    uint8_t frame_type = get_isotp_frame_type(msg);
    uint32_t timestamp = microsecond_timer_get();

    // Determine if this is a request or response based on address
    uint32_t tx_addr;
    uint32_t rx_addr;

    if ((msg->addr >= 0x7E8u) && (msg->addr <= 0x7EFu)) {
      // Standard response address
      rx_addr = msg->addr;
      tx_addr = msg->addr - 8u; // Corresponding request address
    } else if ((msg->addr >= 0x7E0u) && (msg->addr <= 0x7E7u)) {
      // Standard request address
      tx_addr = msg->addr;
      rx_addr = msg->addr + 8u; // Corresponding response address
    } else if (msg->addr == 0x7DFu) {
      // Functional request address
      tx_addr = msg->addr;
      rx_addr = 0u; // Will be determined by response
    } else {
      // Vehicle-specific or extended addressing
      tx_addr = msg->addr;
      rx_addr = msg->addr;
    }

    if (frame_type == ISOTP_SINGLE_FRAME) {
      uint8_t data_length = msg->data[0] & 0x0Fu;
      if ((data_length > 0u) && (data_length <= 7u)) {
        uds_session_t *session = find_or_create_session(tx_addr, rx_addr, msg->bus);
        if (session != NULL) {
          session->total_length = data_length;
          session->received_length = data_length;
          session->last_timestamp = timestamp;
          (void)memcpy(session->data, &msg->data[1], data_length);
          parse_and_callback_uds_message(session);
          session->active = false; // Single frame complete
        }
      }
    } else if (frame_type == ISOTP_FIRST_FRAME) {
      uint16_t total_length = get_isotp_data_length(msg);
      if (total_length > 7u) {
        uds_session_t *session = find_or_create_session(tx_addr, rx_addr, msg->bus);
        if (session != NULL) {
          session->total_length = total_length;
          session->received_length = 6u; // First frame contains 6 bytes of data
          session->sequence_number = 1u;
          session->last_timestamp = timestamp;
          (void)memcpy(session->data, &msg->data[2], 6);
        }
      }
    } else if (frame_type == ISOTP_CONSECUTIVE_FRAME) {
      uint8_t sequence_number = msg->data[0] & 0x0Fu;

      // Find matching session
      for (int i = 0; i < MAX_UDS_SESSIONS; i++) {
        if (uds_sessions[i].active &&
            (uds_sessions[i].tx_addr == tx_addr) &&
            (uds_sessions[i].rx_addr == rx_addr) &&
            (uds_sessions[i].bus == msg->bus) &&
            (uds_sessions[i].sequence_number == sequence_number)) {
          uds_session_t *session = &uds_sessions[i];
          uint8_t data_to_copy = MIN(7, session->total_length - session->received_length);

          if (data_to_copy > 0u) {
            (void)memcpy(&session->data[session->received_length], &msg->data[1], data_to_copy);
            session->received_length += data_to_copy;
            session->sequence_number = (session->sequence_number + 1u) & 0x0Fu;
            session->last_timestamp = timestamp;

            // Check if message is complete
            if (session->received_length >= session->total_length) {
              parse_and_callback_uds_message(session);
              session->active = false; // Message complete
            }
          }
          break;
        }
      }
    } else {
      // Unsupported frame type (e.g., Flow Control), ignore
    }
  }
}

void uds_sniffer_tick(void) {
  if (uds_sniffer_enabled) {
    uint32_t current_time = microsecond_timer_get();
    const uint32_t SESSION_TIMEOUT = 5000000U; // 5 seconds

    // Clean up old sessions
    for (int i = 0; i < MAX_UDS_SESSIONS; i++) {
      if (uds_sessions[i].active &&
          get_ts_elapsed(current_time, uds_sessions[i].last_timestamp) > SESSION_TIMEOUT) {
        uds_sessions[i].active = false;
      }
    }
  }
}
