#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/hyundai_common.h"

#define HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(bus) \
  {0x1CF, bus, 8, .check_relay = false},  /* CRUISE_BUTTON */   \

#define HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x50,  a_can, 16, .check_relay = (a_can) == 0},  /* LKAS */      \
  {0x2A4, a_can, 24, .check_relay = (a_can) == 0},  /* CAM_0x2A4 */ \

#define HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x110, a_can, 32, .check_relay = (a_can) == 0},  /* LKAS_ALT */  \
  {0x362, a_can, 32, .check_relay = (a_can) == 0},  /* CAM_0x362 */ \

#define HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e_can)  \
  {0x12A, e_can, 16, .check_relay = (e_can) == 0},  /* LFA */            \
  {0x1E0, e_can, 16, .check_relay = (e_can) == 0},  /* LFAHDA_CLUSTER */ \

#define HYUNDAI_CANFD_LFA_CAMERA_SYNC_COMMON_TX_MSGS(e_can) \
  {HYUNDAI_CANFD_LFA_COMMAND_ADDR, e_can, 8, .check_relay = false},  /* internal LFA command */ \
  /* Keep a relay-check entry so Panda's MADS state updates on RX, but do not block the physical camera 0x1E0. */ \
  {0x1E0, e_can, 16, .check_relay = true, .disable_static_blocking = true},  /* LFAHDA_CLUSTER */ \

#define HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(e_can, longitudinal) \
  {0x1A0, e_can, 32, .check_relay = (longitudinal)},  /* SCC_CONTROL */ \

// *** Addresses checked in rx hook ***
// EV, ICE, HYBRID: ACCELERATOR (0x35), ACCELERATOR_BRAKE_ALT (0x100), ACCELERATOR_ALT (0x105)
#define HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                          \
  {.msg = {{0x35, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                  \
           {0x100, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                 \
           {0x105, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}}},               \
  {.msg = {{0x175, (pt_bus), 24, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xa0, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xea, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(pt_bus)                                                                                            \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                                       \
  {.msg = {{0x1cf, (pt_bus), 8, 50U, .ignore_checksum = true, .max_counter = 0xfU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(pt_bus)                                                                                              \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                                         \
  {.msg = {{0x1aa, (pt_bus), 16, 50U, .ignore_checksum = true, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

// SCC_CONTROL (from ADAS unit or camera)
#define HYUNDAI_CANFD_SCC_ADDR_CHECK(scc_bus)                                                                            \
  {.msg = {{0x1a0, (scc_bus), 32, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_LFA_CAMERA_SYNC_RX_CHECKS \
  {.msg = {{0x12a, 2, 16, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \

#define HYUNDAI_CANFD_LFA_COMMAND_ADDR 0x7FFU
#define HYUNDAI_CANFD_LFA_COMMAND_MAGIC 0xA5U
#define HYUNDAI_CANFD_LFA_COMMAND_TIMEOUT 50000U

static bool hyundai_canfd_alt_buttons = false;
static bool hyundai_canfd_lka_steer_msg_alt = false;
static bool hyundai_canfd_lfa_camera_sync = false;
static int hyundai_canfd_lfa_command_torque = 0;
static bool hyundai_canfd_lfa_command_request = false;
static bool hyundai_canfd_lfa_ui_active = false;
static bool hyundai_canfd_lfa_command_valid = false;
static uint32_t hyundai_canfd_lfa_command_ts = 0U;
static bool hyundai_canfd_lfa_camera_requested = false;
static float hyundai_canfd_lfa_filtered_speed_kph = 0.0F;
static uint8_t hyundai_canfd_lfa_damping = 10U;
static bool hyundai_canfd_lfa_speed_valid = false;

static bool hyundai_canfd_lfa_command_fresh(void) {
  const uint32_t command_age = safety_get_ts_elapsed(microsecond_timer_get(), hyundai_canfd_lfa_command_ts);
  return hyundai_canfd_lfa_command_valid && (command_age < HYUNDAI_CANFD_LFA_COMMAND_TIMEOUT);
}

static bool hyundai_canfd_checksum_valid(const CANPacket_t *msg) {
  const uint16_t checksum = msg->data[0] | (msg->data[1] << 8U);
  return checksum == hyundai_common_canfd_compute_checksum(msg);
}

static void hyundai_canfd_update_checksum(CANPacket_t *msg) {
  const uint16_t checksum = hyundai_common_canfd_compute_checksum(msg);
  msg->data[0] = checksum & 0xFFU;
  msg->data[1] = checksum >> 8U;
}

static int hyundai_canfd_get_lfa_torque(const CANPacket_t *msg) {
  return (((msg->data[6] & 0xFU) << 7U) | (msg->data[5] >> 1U)) - 1024U;
}

static uint8_t hyundai_canfd_get_lfa_request(const CANPacket_t *msg) {
  return (msg->data[6] >> 4U) & 0x3U;
}

static uint8_t hyundai_canfd_get_lfa_damping(void) {
  // The camera uses the 100 Hz four-wheel average through an approximately
  // 100 ms first-order filter. The small offset aligns the measured integer
  // transition thresholds across independent captures.
  const float speed_kph = hyundai_canfd_lfa_filtered_speed_kph + 0.045F;
  float damping;

  if (speed_kph <= 30.0F) {
    damping = 10.0F;
  } else if (speed_kph <= 40.0F) {
    damping = 10.0F + (1.5F * (speed_kph - 30.0F));
  } else if (speed_kph <= 50.0F) {
    damping = 25.0F + (speed_kph - 40.0F);
  } else if (speed_kph <= 60.0F) {
    damping = 35.0F + (0.5F * (speed_kph - 50.0F));
  } else if (speed_kph <= 100.0F) {
    damping = speed_kph - 20.0F;
  } else if (speed_kph <= 108.0F) {
    damping = 80.0F + (0.25F * (speed_kph - 100.0F));
  } else {
    damping = 82.0F;
  }

  // Positive float-to-integer conversion intentionally truncates, matching
  // the camera's observed staircase rather than rounding to nearest.
  return (uint8_t)SAFETY_CLAMP((int)damping, 10, 82);
}

static void hyundai_canfd_reset_lfa_safety_state(void) {
  const uint32_t ts = microsecond_timer_get();

  desired_torque_last = 0;
  rt_torque_last = 0;
  ts_torque_check_last = ts;
  valid_steer_req_count = 0;
  invalid_steer_req_count = 0;
  ts_steer_req_mismatch_last = ts;
}

static unsigned int hyundai_canfd_get_lka_addr(void) {
  return hyundai_canfd_lka_steer_msg_alt ? 0x110U : 0x50U;
}

static uint8_t hyundai_canfd_get_counter(const CANPacket_t *msg) {
  uint8_t ret = 0;
  if (GET_LEN(msg) == 8U) {
    ret = msg->data[1] >> 4;
  } else {
    ret = msg->data[2];
  }
  return ret;
}

static uint32_t hyundai_canfd_get_checksum(const CANPacket_t *msg) {
  uint32_t chksum = msg->data[0] | (msg->data[1] << 8);
  return chksum;
}

static void hyundai_canfd_rx_hook(const CANPacket_t *msg) {

  const unsigned pt_bus = hyundai_canfd_lka_steer_msg ? 1U : 0U;
  const unsigned int scc_bus = hyundai_camera_scc ? 2U : pt_bus;

  if (msg->bus == pt_bus) {
    // driver torque
    if (msg->addr == 0xeaU) {
      int torque_driver_new = ((msg->data[11] & 0x1fU) << 8U) | msg->data[10];
      torque_driver_new -= 4095;
      update_sample(&torque_driver, torque_driver_new);
    }

    // cruise buttons
    const unsigned int button_addr = hyundai_canfd_alt_buttons ? 0x1aaU : 0x1cfU;
    if (msg->addr == button_addr) {
      bool main_button = false;
      int cruise_button = 0;
      if (msg->addr == 0x1cfU) {
        cruise_button = msg->data[2] & 0x7U;
        main_button = GET_BIT(msg, 19U);
        mads_button_press = GET_BIT(msg, 23U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      } else {
        cruise_button = (msg->data[4] >> 4) & 0x7U;
        main_button = GET_BIT(msg, 34U);
        mads_button_press = GET_BIT(msg, 39U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      }
      hyundai_common_cruise_buttons_check(cruise_button, main_button);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((msg->addr == 0x35U) && hyundai_ev_gas_signal) {
      gas_pressed = msg->data[5] != 0U;
    } else if ((msg->addr == 0x105U) && hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 103U) || (msg->data[13] != 0U) || GET_BIT(msg, 112U);
    } else if ((msg->addr == 0x100U) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 176U);
    } else {
    }

    // brake press
    if (msg->addr == 0x175U) {
      brake_pressed = GET_BIT(msg, 81U);
    }

    // vehicle moving
    if (msg->addr == 0xa0U) {
      uint32_t fl = (GET_BYTES(msg, 8, 2)) & 0x3FFFU;
      uint32_t fr = (GET_BYTES(msg, 10, 2)) & 0x3FFFU;
      uint32_t rl = (GET_BYTES(msg, 12, 2)) & 0x3FFFU;
      uint32_t rr = (GET_BYTES(msg, 14, 2)) & 0x3FFFU;
      vehicle_moving = (fl > HYUNDAI_STANDSTILL_THRSLD) || (fr > HYUNDAI_STANDSTILL_THRSLD) ||
                       (rl > HYUNDAI_STANDSTILL_THRSLD) || (rr > HYUNDAI_STANDSTILL_THRSLD);

      // average of all 4 wheel speeds. Conversion: raw * 0.03125 / 3.6 = m/s
      UPDATE_VEHICLE_SPEED((fr + rr + rl + fl) / 4.0 * 0.03125 * KPH_TO_MS);

      const float wheel_speed_kph = (fr + rr + rl + fl) / 4.0F * 0.03125F;
      if (!hyundai_canfd_lfa_speed_valid) {
        hyundai_canfd_lfa_filtered_speed_kph = wheel_speed_kph;
        hyundai_canfd_lfa_speed_valid = true;
      } else {
        hyundai_canfd_lfa_filtered_speed_kph += 0.1F * (wheel_speed_kph - hyundai_canfd_lfa_filtered_speed_kph);
      }
    }
  }

  if (msg->bus == scc_bus) {
    // cruise state
    if ((msg->addr == 0x1a0U) && !hyundai_longitudinal) {
      // 1=enabled, 2=driver override
      int cruise_status = ((msg->data[8] >> 4) & 0x7U);
      bool cruise_engaged = (cruise_status == 1) || (cruise_status == 2);
      hyundai_common_cruise_state_check(cruise_engaged);
      acc_main_on = GET_BIT(msg, 66U);
    }
  }

  hyundai_common_reset_acc_main_on_mismatches();
}

static bool hyundai_canfd_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits HYUNDAI_CANFD_STEERING_LIMITS = {
    .max_torque = hyundai_canfd_lfa_camera_sync ? 384 : 270,
    .max_rt_delta = 112,
    .max_rate_up = hyundai_canfd_lfa_camera_sync ? 4 : 2,
    .max_rate_down = hyundai_canfd_lfa_camera_sync ? 7 : 3,
    .driver_torque_allowance = 250,
    .driver_torque_multiplier = 2,
    .type = TorqueDriverLimited,

    // the EPS faults when the steering angle is above a certain threshold for too long. to prevent this,
    // we allow setting torque actuation bit to 0 while maintaining the requested torque value for two consecutive frames
    .min_valid_request_frames = 89,
    .max_invalid_request_frames = 2,
    .min_valid_request_rt_interval = 810000,  // 810ms; a ~10% buffer on cutting every 90 frames
    .has_steer_req_tolerance = true,
  };

  bool tx = true;

  // LFA camera-sync mode uses a virtual command which Panda consumes internally. The real
  // camera LFA is modified later in the hardware forwarding path, preserving its exact cadence.
  if (hyundai_canfd_lfa_camera_sync && (msg->addr == HYUNDAI_CANFD_LFA_COMMAND_ADDR)) {
    const uint8_t request = msg->data[2];
    const uint8_t ui_active = msg->data[4];
    const int desired_torque = to_signed(GET_BYTES(msg, 0, 2), 16);
    const bool command_format_valid = (msg->data[3] == HYUNDAI_CANFD_LFA_COMMAND_MAGIC) &&
                                      (request <= 1U) && (ui_active <= 1U) && (GET_BYTES(msg, 5, 3) == 0U);

    // A malformed or unsafe update fails closed. Stock camera steering is never restored.
    hyundai_canfd_lfa_command_torque = 0;
    hyundai_canfd_lfa_command_request = false;
    hyundai_canfd_lfa_ui_active = false;
    hyundai_canfd_lfa_command_valid = false;
    safety_tx_consumed = true;

    tx = command_format_valid && !steer_torque_cmd_checks(desired_torque, request == 1U, HYUNDAI_CANFD_STEERING_LIMITS);

    if (tx) {
      hyundai_canfd_lfa_command_torque = desired_torque;
      hyundai_canfd_lfa_command_request = request == 1U;
      hyundai_canfd_lfa_ui_active = ui_active == 1U;
      hyundai_canfd_lfa_command_valid = true;
      hyundai_canfd_lfa_command_ts = microsecond_timer_get();
    }
  }

  // steering messages sent directly by openpilot on all other CAN-FD configurations
  const unsigned int steer_addr = (hyundai_canfd_lka_steer_msg && !hyundai_longitudinal) ? hyundai_canfd_get_lka_addr() : 0x12aU;
  if (!hyundai_canfd_lfa_camera_sync && (msg->addr == steer_addr)) {
    int desired_torque = hyundai_canfd_get_lfa_torque(msg);
    bool steer_req = GET_BIT(msg, 52U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, HYUNDAI_CANFD_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // cruise buttons check
  if (msg->addr == 0x1cfU) {
    int button = msg->data[2] & 0x7U;
    bool is_cancel = (button == HYUNDAI_BTN_CANCEL);
    bool is_resume = (button == HYUNDAI_BTN_RESUME);
    bool is_set = (button == HYUNDAI_BTN_SET);

    bool allowed = (is_cancel && cruise_engaged_prev) || ((is_resume || is_set) && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  // UDS: only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (((msg->addr == 0x730U) && hyundai_canfd_lka_steer_msg) || ((msg->addr == 0x7D0U) && !hyundai_camera_scc)) {
    if ((GET_BYTES(msg, 0, 4) != 0x00803E02U) || (GET_BYTES(msg, 4, 4) != 0x0U)) {
      tx = false;
    }
  }

  // ACCEL: safety check
  if (msg->addr == 0x1a0U) {
    int desired_accel_raw = (((msg->data[17] & 0x7U) << 8) | msg->data[16]) - 1023U;
    int desired_accel_val = ((msg->data[18] << 4) | (msg->data[17] >> 4)) - 1023U;

    bool violation = false;

    if (hyundai_longitudinal) {
      violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
    } else {
      // only used to cancel on here
      const int acc_mode = (msg->data[8] >> 4) & 0x7U;
      if (acc_mode != 4) {
        violation = true;
      }

      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
        violation = true;
      }
    }

    if (violation) {
      tx = false;
    }

    acc_main_on_tx = GET_BIT(msg, 66U);
    hyundai_common_acc_main_on_sync();
  }

  return tx;
}

static bool hyundai_canfd_fwd_hook(int bus_num, int addr) {
  // The physical camera LFA travels camera bus 2 -> car bus 0. Never allow a bus 0
  // copy to travel back toward the camera in camera-sync mode.
  return hyundai_canfd_lfa_camera_sync && (bus_num == 0) && (addr == 0x12A);
}

static void hyundai_canfd_fwd_modify(CANPacket_t *msg) {
  const bool is_camera_lfa = hyundai_canfd_lfa_camera_sync && (msg->bus == 2U) &&
                             (msg->addr == 0x12AU) && (GET_LEN(msg) == 16U);
  if (is_camera_lfa) {
    hyundai_canfd_lfa_camera_requested = hyundai_canfd_checksum_valid(msg) &&
                                         (hyundai_canfd_get_lfa_request(msg) == 1U);

    const bool command_allowed = hyundai_canfd_lfa_command_fresh() &&
                                 (controls_allowed || controls_allowed_lateral);
    const int desired_torque = command_allowed ? hyundai_canfd_lfa_command_torque : 0;
    const bool steer_req = command_allowed && hyundai_canfd_lfa_command_request;
    const uint16_t raw_torque = (uint16_t)(desired_torque + 1024);

    if (!command_allowed) {
      hyundai_canfd_lfa_command_valid = false;
      hyundai_canfd_reset_lfa_safety_state();
    }

    // The camera supplies the physical 100 Hz timing, counter, and unrelated payload.
    // Panda owns only the rack-facing torque request for the entire camera-sync mode.
    msg->data[5] = (msg->data[5] & 0x1U) | ((raw_torque & 0x7FU) << 1U);
    msg->data[6] = (msg->data[6] & 0xC0U) | ((raw_torque >> 7U) & 0xFU) | (steer_req ? 0x10U : 0U);

    const bool steering_active = command_allowed && (steer_req || (desired_torque != 0));
    if (!steering_active) {
      msg->data[13] = 100U;
    } else if (!hyundai_canfd_lfa_camera_requested) {
      // Stock damping changes on even counters and is held on the odd frame.
      if (!hyundai_canfd_lfa_speed_valid || ((msg->data[2] & 1U) == 0U)) {
        hyundai_canfd_lfa_damping = hyundai_canfd_get_lfa_damping();
      }
      msg->data[13] = hyundai_canfd_lfa_damping;
    }

    hyundai_canfd_update_checksum(msg);
  }

  // Keep the cluster's LFA ownership indicator aligned with openpilot. Preserve the
  // camera's physical cadence and every unrelated UI, alert, and lane signal.
  const bool is_camera_ui = hyundai_canfd_lfa_camera_sync && (msg->bus == 2U) &&
                            (msg->addr == 0x161U) && (GET_LEN(msg) == 32U);
  if (is_camera_ui && hyundai_canfd_checksum_valid(msg)) {
    const bool ui_active = hyundai_canfd_lfa_command_fresh() &&
                           (controls_allowed || controls_allowed_lateral) && hyundai_canfd_lfa_ui_active;
    msg->data[8] = (msg->data[8] & 0xFCU) | (ui_active ? 1U : 0U);   // CENTERLINE
    msg->data[28] = (msg->data[28] & 0xF0U) | (ui_active ? 2U : 0U); // LFA_ICON

    // The camera's stock hands-off escalation cancels SCC even though openpilot
    // remains the steering owner. Suppress only the observed three-stage sequence.
    const uint8_t alerts_2 = (msg->data[16] >> 6U) | ((msg->data[17] & 0x7U) << 2U);
    const uint8_t alerts_5 = msg->data[19] & 0x1FU;
    const uint8_t sounds_2 = msg->data[20] >> 4U;
    const bool hands_off_warning = ((alerts_2 == 1U) && (sounds_2 == 0U)) ||
                                   ((alerts_2 == 2U) && (sounds_2 == 3U)) ||
                                   ((alerts_2 == 4U) && (sounds_2 == 6U)) || (alerts_5 == 1U);
    if (ui_active && hands_off_warning) {
      msg->data[16] &= 0x3FU;
      msg->data[17] &= 0xF8U;
      msg->data[19] &= 0xE0U;
      msg->data[20] &= 0x0FU;
    }
    hyundai_canfd_update_checksum(msg);
  }

  // Give the camera an acknowledgement matching its original request. The car-side
  // MDPS frame stays honest, and every other camera-facing MDPS signal stays physical.
  const bool is_mdps_to_camera = hyundai_canfd_lfa_camera_sync && (msg->bus == 0U) &&
                                 (msg->addr == 0xEAU) && (GET_LEN(msg) == 24U);
  if (is_mdps_to_camera && hyundai_canfd_checksum_valid(msg)) {
    msg->data[6] = (msg->data[6] & 0xFCU) | (hyundai_canfd_lfa_camera_requested ? 1U : 0U);
    hyundai_canfd_update_checksum(msg);
  }
}

static safety_config hyundai_canfd_init(uint16_t param) {
  const uint16_t HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT = 128;
  const uint16_t HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32;
  const uint16_t HYUNDAI_PARAM_CANFD_LFA_CAMERA_SYNC = 1024;

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(1)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(1, true)
    {0x51,  0, 32, .check_relay = false},  // ADRV_0x51
    {0x730, 1,  8, .check_relay = false},  // tester present for ADAS ECU disable
    {0x160, 1, 16, .check_relay = false},  // ADRV_0x160
    {0x1EA, 1, 32, .check_relay = false},  // ADRV_0x1ea
    {0x200, 1,  8, .check_relay = false},  // ADRV_0x200
    {0x345, 1,  8, .check_relay = false},  // ADRV_0x345
    {0x1DA, 1, 32, .check_relay = false},  // ADRV_0x1da
  };

  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
  };

  // ADRV_0x160 is checked for radar liveness
  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, true)
    {0x160, 0, 16, .check_relay = true}, // ADRV_0x160
    {0x7D0, 0, 8, .check_relay = false},  // tester present for radar ECU disable
  };

  // ADRV_0x160 is checked for relay malfunction
#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0x160, 0, 16, .check_relay = (longitudinal)}, /* ADRV_0x160 */ \

#define HYUNDAI_CANFD_LFA_CAMERA_SYNC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_CAMERA_SYNC_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0x160, 0, 16, .check_relay = (longitudinal)}, /* ADRV_0x160 */ \

  hyundai_common_init(param);

  gen_crc_lookup_table_16(0x1021, hyundai_canfd_crc_lut);
  hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
  hyundai_canfd_lka_steer_msg_alt = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT);
  hyundai_canfd_lfa_camera_sync = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LFA_CAMERA_SYNC);
  hyundai_canfd_lfa_command_torque = 0;
  hyundai_canfd_lfa_command_request = false;
  hyundai_canfd_lfa_ui_active = false;
  hyundai_canfd_lfa_command_valid = false;
  hyundai_canfd_lfa_command_ts = 0U;
  hyundai_canfd_lfa_camera_requested = false;
  hyundai_canfd_lfa_filtered_speed_kph = 0.0F;
  hyundai_canfd_lfa_damping = 10U;
  hyundai_canfd_lfa_speed_valid = false;

  safety_config ret;
  if (hyundai_longitudinal) {
    if (hyundai_canfd_lka_steer_msg) {
      static RxCheck hyundai_canfd_lka_steer_msg_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
      };

      ret = BUILD_SAFETY_CFG(hyundai_canfd_lka_steer_msg_long_rx_checks, HYUNDAI_CANFD_LKA_STEER_MSG_LONG_TX_MSGS);

    } else {
      // Longitudinal checks for LFA steering
      static RxCheck hyundai_canfd_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_long_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
      };

      static RxCheck hyundai_canfd_lfa_camera_sync_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_LFA_CAMERA_SYNC_RX_CHECKS
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(true)
      };

      static CanMsg hyundai_canfd_lfa_camera_sync_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_CAMERA_SYNC_TX_MSGS(true)
      };

      if (hyundai_canfd_lfa_camera_sync) {
        SET_RX_CHECKS(hyundai_canfd_lfa_camera_sync_long_rx_checks, ret);
      } else if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_long_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_long_rx_checks, ret);
      }

      if (hyundai_camera_scc) {
        if (hyundai_canfd_lfa_camera_sync) {
          SET_TX_MSGS(hyundai_canfd_lfa_camera_sync_tx_msgs, ret);
        } else {
          SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
        }
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS, ret);
      }
    }

  } else {
    if (hyundai_canfd_lka_steer_msg) {
      // *** LKA steering checks ***
      // E-CAN is on bus 1, SCC messages are sent on cars with ADRV ECU.
      // Does not use the alt buttons message
      static RxCheck hyundai_canfd_lka_steer_msg_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(1)
      };

      SET_RX_CHECKS(hyundai_canfd_lka_steer_msg_rx_checks, ret);
      if (hyundai_canfd_lka_steer_msg_alt) {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS, ret);
      }

    } else if (!hyundai_camera_scc) {
      // Radar sends SCC messages on these cars instead of camera
      static RxCheck hyundai_canfd_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_TX_MSGS, ret);

      if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_radar_scc_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_radar_scc_rx_checks, ret);
      }

    } else {
      // *** LFA steering checks ***
      // Camera sends SCC messages on LFA steering cars.
      // Both button messages exist on some platforms, so we ensure we track the correct one using flag
      static RxCheck hyundai_canfd_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_alt_buttons_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_lfa_camera_sync_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
        HYUNDAI_CANFD_LFA_CAMERA_SYNC_RX_CHECKS
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(false)
      };

      static CanMsg hyundai_canfd_lfa_camera_sync_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_CAMERA_SYNC_TX_MSGS(false)
      };

      if (hyundai_canfd_lfa_camera_sync) {
        SET_TX_MSGS(hyundai_canfd_lfa_camera_sync_tx_msgs, ret);
      } else {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
      }

      if (hyundai_canfd_lfa_camera_sync) {
        SET_RX_CHECKS(hyundai_canfd_lfa_camera_sync_rx_checks, ret);
      } else if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_rx_checks, ret);
      }
    }
  }

  return ret;
}

const safety_hooks hyundai_canfd_hooks = {
  .init = hyundai_canfd_init,
  .rx = hyundai_canfd_rx_hook,
  .tx = hyundai_canfd_tx_hook,
  .fwd = hyundai_canfd_fwd_hook,
  .fwd_modify = hyundai_canfd_fwd_modify,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
