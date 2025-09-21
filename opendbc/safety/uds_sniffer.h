#pragma once

#include "opendbc/safety/uds_sniffer_declarations.h"
#include "stddef.h"


// Global UDS sniffer state
uds_session_t uds_sessions[MAX_UDS_SESSIONS];
uds_message_callback_t uds_callback = NULL;
bool uds_sniffer_enabled = false;

void *memcpy(void *dest, const void *src, unsigned int len);
void *memset(void *str, int c, unsigned int n);


// UDS Service name lookup table
static const struct {
  uint8_t service_id;
  const char* name;
} uds_service_names[] = {
  {UDS_SERVICE_DIAGNOSTIC_SESSION_CONTROL, "DiagnosticSessionControl"},
  {UDS_SERVICE_ECU_RESET, "ECUReset"},
  {UDS_SERVICE_SECURITY_ACCESS, "SecurityAccess"},
  {UDS_SERVICE_TESTER_PRESENT, "TesterPresent"},
  {UDS_SERVICE_READ_DATA_BY_IDENTIFIER, "ReadDataByIdentifier"},
  {UDS_SERVICE_WRITE_DATA_BY_IDENTIFIER, "WriteDataByIdentifier"},
  {UDS_SERVICE_ROUTINE_CONTROL, "RoutineControl"},
  {UDS_SERVICE_READ_DTC_INFORMATION, "ReadDTCInformation"},
  {UDS_SERVICE_CLEAR_DIAGNOSTIC_INFORMATION, "ClearDiagnosticInformation"},
};

// UDS Data Identifier name lookup table
static const struct {
  uint16_t did;
  const char* name;
} uds_did_names[] = {
  {UDS_DID_VIN, "VIN"},
  {UDS_DID_ECU_SOFTWARE_NUMBER, "ECU_Software_Number"},
  {UDS_DID_ECU_SOFTWARE_VERSION, "ECU_Software_Version"},
  {UDS_DID_ECU_HARDWARE_NUMBER, "ECU_Hardware_Number"},
  {UDS_DID_ECU_SERIAL_NUMBER, "ECU_Serial_Number"},
  {UDS_DID_ACTIVE_DIAGNOSTIC_SESSION, "Active_Diagnostic_Session"},
  {0xF180, "Boot_Software_Identification"},
  {0xF181, "Application_Software_Identification"},
  {0xF182, "Application_Data_Identification"},
  {0xF183, "Boot_Software_Fingerprint"},
  {0xF184, "Application_Software_Fingerprint"},
  {0xF185, "Application_Data_Fingerprint"},
  {0xF187, "Vehicle_Manufacturer_Spare_Part_Number"},
  {0xF18A, "System_Supplier_Identifier"},
  {0xF18B, "ECU_Manufacturing_Date"},
  {0xF18D, "Supported_Functional_Units"},
  {0xF18E, "Vehicle_Manufacturer_Kit_Assembly_Part_Number"},
  {0xF192, "System_Supplier_ECU_Hardware_Number"},
  {0xF193, "System_Supplier_ECU_Hardware_Version_Number"},
  {0xF194, "System_Supplier_ECU_Software_Number"},
  {0xF195, "System_Supplier_ECU_Software_Version_Number"},
  {0xF196, "Exhaust_Regulation_Or_Type_Approval_Number"},
  {0xF197, "System_Name_Or_Engine_Type"},
  {0xF198, "Repair_Shop_Code_Or_Tester_Serial_Number"},
  {0xF199, "Programming_Date"},
  {0xF19A, "Calibration_Repair_Shop_Code_Or_Calibration_Equipment_Serial_Number"},
  {0xF19B, "Calibration_Date"},
  {0xF19C, "Calibration_Equipment_Software_Number"},
  {0xF19D, "ECU_Installation_Date"},
  {0xF19E, "ODX_File"},
  {0xF19F, "Entity"},
};

// UDS Negative Response Code lookup table
static const struct {
  uint8_t nrc;
  const char* name;
} uds_nrc_names[] = {
  {0x10, "General_Reject"},
  {0x11, "Service_Not_Supported"},
  {0x12, "Sub_Function_Not_Supported"},
  {0x13, "Incorrect_Message_Length_Or_Invalid_Format"},
  {0x14, "Response_Too_Long"},
  {0x21, "Busy_Repeat_Request"},
  {0x22, "Conditions_Not_Correct"},
  {0x24, "Request_Sequence_Error"},
  {0x25, "No_Response_From_Subnet_Component"},
  {0x26, "Failure_Prevents_Execution_Of_Requested_Action"},
  {0x31, "Request_Out_Of_Range"},
  {0x33, "Security_Access_Denied"},
  {0x35, "Invalid_Key"},
  {0x36, "Exceed_Number_Of_Attempts"},
  {0x37, "Required_Time_Delay_Not_Expired"},
  {0x70, "Upload_Download_Not_Accepted"},
  {0x71, "Transfer_Data_Suspended"},
  {0x72, "General_Programming_Failure"},
  {0x73, "Wrong_Block_Sequence_Counter"},
  {0x78, "Request_Correctly_Received_Response_Pending"},
  {0x7E, "Sub_Function_Not_Supported_In_Active_Session"},
  {0x7F, "Service_Not_Supported_In_Active_Session"},
};

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
  uds_sniffer_enabled = false;
}

void uds_sniffer_set_callback(uds_message_callback_t callback) {
  uds_callback = callback;
}

void uds_sniffer_enable(bool enable) {
  uds_sniffer_enabled = enable;
}

bool is_uds_address(uint32_t addr) {
  // Standard UDS addresses
  // Physical addressing: 0x7E0-0x7E7 (request), 0x7E8-0x7EF (response)
  // Functional addressing: 0x7DF (request)
  // Extended addressing: 0x18DAxxxx, 0x18DBxxxx

  if ((addr >= 0x7E0 && addr <= 0x7EF) || addr == 0x7DF) {
    return true;
  }

  // Extended addressing (29-bit CAN IDs)
  if ((addr & 0xFFFF0000) == 0x18DA0000 || (addr & 0xFFFF0000) == 0x18DB0000) {
    return true;
  }

  // Vehicle-specific UDS addresses (Hyundai examples)
  if (addr == 0x730 || addr == 0x7D0 || addr == 0x740 || addr == 0x7A0 || addr == 0x7CC) {
    return true;
  }

  return false;
}

bool is_isotp_frame(const CANPacket_t *msg) {
  if (GET_LEN(msg) == 0) {
    return false;
  }

  uint8_t pci = msg->data[0] >> 4;
  return (pci <= ISOTP_FLOW_CONTROL_FRAME);
}

uint8_t get_isotp_frame_type(const CANPacket_t *msg) {
  return (msg->data[0] >> 4) & 0x0F;
}

uint16_t get_isotp_data_length(const CANPacket_t *msg) {
  uint8_t frame_type = get_isotp_frame_type(msg);

  if (frame_type == ISOTP_SINGLE_FRAME) {
    return msg->data[0] & 0x0F;
  } else if (frame_type == ISOTP_FIRST_FRAME) {
    return ((msg->data[0] & 0x0F) << 8) | msg->data[1];
  }

  return 0;
}

static uds_session_t* find_or_create_session(uint32_t tx_addr, uint32_t rx_addr, uint8_t bus) {
  uds_session_t* free_session = NULL;
  uint32_t oldest_timestamp = UINT32_MAX;
  uds_session_t* oldest_session = NULL;

  // First, try to find existing session
  for (int i = 0; i < MAX_UDS_SESSIONS; i++) {
    if (uds_sessions[i].active &&
        uds_sessions[i].tx_addr == tx_addr &&
        uds_sessions[i].rx_addr == rx_addr &&
        uds_sessions[i].bus == bus) {
      return &uds_sessions[i];
    }

    if (!uds_sessions[i].active && free_session == NULL) {
      free_session = &uds_sessions[i];
    }

    if (uds_sessions[i].last_timestamp < oldest_timestamp) {
      oldest_timestamp = uds_sessions[i].last_timestamp;
      oldest_session = &uds_sessions[i];
    }
  }

  // Use free session if available, otherwise reuse oldest
  uds_session_t* session = free_session ? free_session : oldest_session;

  if (session) {
    session->active = true;
    session->tx_addr = tx_addr;
    session->rx_addr = rx_addr;
    session->bus = bus;
    session->sequence_number = 0;
    session->total_length = 0;
    session->received_length = 0;
    session->last_timestamp = microsecond_timer_get();
  }

  return session;
}

static void parse_and_callback_uds_message(uds_session_t* session) {
  if (!session || session->received_length < 1 || !uds_callback) {
    return;
  }

  uds_message_t msg = {0};
  msg.timestamp = session->last_timestamp;

  uint8_t* data = session->data;
  char service_id = data[0];

  // Check if it's a negative response
  if (service_id == UDS_RESPONSE_NEGATIVE) {
    if (session->received_length >= 3) {
      msg.is_negative_response = true;
      msg.service_id = data[1];  // Original service ID
      msg.negative_response_code = data[2];
      msg.data_length = session->received_length - 3;
      if (msg.data_length > 0) {
        memcpy(msg.data, &data[3], MIN(msg.data_length, MAX_UDS_DATA_SIZE));
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
    if ((msg.service_id == UDS_SERVICE_READ_DATA_BY_IDENTIFIER ||
         msg.service_id == UDS_SERVICE_WRITE_DATA_BY_IDENTIFIER) &&
        session->received_length >= 3) {
      msg.data_identifier = (data[1] << 8) | data[2];
      msg.data_length = (uint8_t)(session->received_length - 3);
      if (msg.data_length > 0) {
        memcpy(msg.data, &data[3], MIN(msg.data_length, MAX_UDS_DATA_SIZE));
      }
    } else {
      msg.data_length = session->received_length - 1;
      if (msg.data_length > 0) {
        memcpy(msg.data, &data[1], MIN(msg.data_length, MAX_UDS_DATA_SIZE));
      }
    }
  }

  uds_callback(&msg, session->tx_addr, session->rx_addr);
}

bool uds_sniffer_process_message(const CANPacket_t *msg) {
  if (!uds_sniffer_enabled || !is_uds_address(msg->addr) || !is_isotp_frame(msg)) {
    return false;
  }

  uint8_t frame_type = get_isotp_frame_type(msg);
  uint32_t timestamp = microsecond_timer_get();

  // Determine if this is a request or response based on address
  uint32_t tx_addr, rx_addr;

  if (msg->addr >= 0x7E8 && msg->addr <= 0x7EF) {
    // Standard response address
    rx_addr = msg->addr;
    tx_addr = msg->addr - 8;  // Corresponding request address
  } else if (msg->addr >= 0x7E0 && msg->addr <= 0x7E7) {
    // Standard request address
    tx_addr = msg->addr;
    rx_addr = msg->addr + 8;  // Corresponding response address
  } else if (msg->addr == 0x7DF) {
    // Functional request address
    tx_addr = msg->addr;
    rx_addr = 0;  // Will be determined by response
  } else {
    // Vehicle-specific or extended addressing
    tx_addr = msg->addr;
    rx_addr = msg->addr;
  }

  if (frame_type == ISOTP_SINGLE_FRAME) {
    uint8_t data_length = msg->data[0] & 0x0F;
    if (data_length > 0 && data_length <= 7) {
      uds_session_t* session = find_or_create_session(tx_addr, rx_addr, msg->bus);
      if (session) {
        session->total_length = data_length;
        session->received_length = data_length;
        session->last_timestamp = timestamp;
        memcpy(session->data, &msg->data[1], data_length);
        parse_and_callback_uds_message(session);
        session->active = false;  // Single frame complete
      }
    }
  } else if (frame_type == ISOTP_FIRST_FRAME) {
    uint16_t total_length = get_isotp_data_length(msg);
    if (total_length > 7) {
      uds_session_t* session = find_or_create_session(tx_addr, rx_addr, msg->bus);
      if (session) {
        session->total_length = total_length;
        session->received_length = 6;  // First frame contains 6 bytes of data
        session->sequence_number = 1;
        session->last_timestamp = timestamp;
        memcpy(session->data, &msg->data[2], 6);
      }
    }
  } else if (frame_type == ISOTP_CONSECUTIVE_FRAME) {
    uint8_t sequence_number = msg->data[0] & 0x0F;

    // Find matching session
    for (int i = 0; i < MAX_UDS_SESSIONS; i++) {
      if (uds_sessions[i].active &&
          uds_sessions[i].tx_addr == tx_addr &&
          uds_sessions[i].rx_addr == rx_addr &&
          uds_sessions[i].bus == msg->bus &&
          uds_sessions[i].sequence_number == sequence_number) {

        uds_session_t* session = &uds_sessions[i];
        uint8_t data_to_copy = MIN(7, session->total_length - session->received_length);

        if (data_to_copy > 0) {
          memcpy(&session->data[session->received_length], &msg->data[1], data_to_copy);
          session->received_length += data_to_copy;
          session->sequence_number = (session->sequence_number + 1) & 0x0F;
          session->last_timestamp = timestamp;

          // Check if message is complete
          if (session->received_length >= session->total_length) {
            parse_and_callback_uds_message(session);
            session->active = false;  // Message complete
          }
        }
        break;
      }
    }
  }

  return true;
}

void uds_sniffer_tick(void) {
  if (!uds_sniffer_enabled) {
    return;
  }

  uint32_t current_time = microsecond_timer_get();
  const uint32_t SESSION_TIMEOUT = 5000000U;  // 5 seconds

  // Clean up old sessions
  for (int i = 0; i < MAX_UDS_SESSIONS; i++) {
    if (uds_sessions[i].active &&
        get_ts_elapsed(current_time, uds_sessions[i].last_timestamp) > SESSION_TIMEOUT) {
      uds_sessions[i].active = false;
    }
  }
}

const char* get_uds_service_name(uint8_t service_id) {
  for (unsigned int i = 0; i < sizeof(uds_service_names) / sizeof(uds_service_names[0]); i++) {
    if (uds_service_names[i].service_id == service_id) {
      return uds_service_names[i].name;
    }
  }
  return "Unknown_Service";
}

const char* get_uds_did_name(uint16_t did) {
  for (unsigned int i = 0; i < sizeof(uds_did_names) / sizeof(uds_did_names[0]); i++) {
    if (uds_did_names[i].did == did) {
      return uds_did_names[i].name;
    }
  }
  return "Unknown_DID";
}

const char* get_uds_nrc_name(uint8_t nrc) {
  for (unsigned int i = 0; i < sizeof(uds_nrc_names) / sizeof(uds_nrc_names[0]); i++) {
    if (uds_nrc_names[i].nrc == nrc) {
      return uds_nrc_names[i].name;
    }
  }
  return "Unknown_NRC";
}
