#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/chrysler_common.h"

#define FCA_ATLANTIS_ENGINE_2        0xDCU
#define FCA_ATLANTIS_ABS_1           0xEEU
#define FCA_ATLANTIS_ABS_3           0xFAU
#define FCA_ATLANTIS_EPS_2           0x106U
#define FCA_ATLANTIS_LKA_COMMAND     0x1F6U
#define FCA_ATLANTIS_LKA_COMMAND_2   0x117U
#define FCA_ATLANTIS_LKA_HUD_2       0x547U
#define FCA_ATLANTIS_LKA_HUD_3       0x5A2U
#define FCA_ATLANTIS_ACC_2           0x22AU
#define FCA_ATLANTIS_ACC_BUTTON      0x2FAU

// Torque winddown after controls are lost (e.g. brake press). Cutting torque to zero in one frame is suspected of
// latching EPS LKA_FAULT, and stock LKA always ramps down. After a disengage, allow torque with LKA_ACTIVE set that
// never grows in magnitude or changes sign, and stays under a limit that falls at FCA_ATLANTIS_MAX_RATE_DOWN per
// frame from the last torque allowed while engaged. The limit depends only on time, so it ends by itself.
#define FCA_ATLANTIS_MAX_RATE_DOWN 12
#define FCA_ATLANTIS_FRAME_US 10000U  // LKA_COMMAND_2 is sent at 100 Hz
#define FCA_ATLANTIS_WINDDOWN_SLACK_FRAMES 5  // frames of delay tolerated before the limit starts falling

// fca_atlantis torque winddown: the last LKA_COMMAND_2 torque allowed while engaged and when, and the last allowed
// torque during the winddown
static int fca_atlantis_engaged_torque = 0;
static uint32_t fca_atlantis_ts_engaged = 0;
static int fca_atlantis_winddown_torque = 0;
static bool fca_atlantis_in_winddown = false;  // the last allowed LKA_COMMAND_2 was a winddown frame
// fca_atlantis LKA_COMMAND: one 1/4 scale copy of the last LKA_COMMAND_2 may be sent
static bool fca_atlantis_lka_copy_pending = false;
static int fca_atlantis_lka_copy_torque = 0;
static bool fca_atlantis_lka_copy_steer_req = false;

static safety_config fca_atlantis_init(uint16_t param) {
  SAFETY_UNUSED(param);

  fca_atlantis_engaged_torque = 0;
  fca_atlantis_ts_engaged = 0;
  fca_atlantis_winddown_torque = 0;
  fca_atlantis_in_winddown = false;
  fca_atlantis_lka_copy_pending = false;
  fca_atlantis_lka_copy_torque = 0;
  fca_atlantis_lka_copy_steer_req = false;

  static const CanMsg FCA_ATLANTIS_TX_MSGS[] = {
    {FCA_ATLANTIS_LKA_COMMAND, 0, 4, .check_relay = true},
    {FCA_ATLANTIS_LKA_COMMAND_2, 0, 4, .check_relay = true},
    {FCA_ATLANTIS_LKA_HUD_2, 0, 8, .check_relay = true},
    {FCA_ATLANTIS_LKA_HUD_3, 0, 8, .check_relay = true},
    {FCA_ATLANTIS_ACC_BUTTON, 2, 4, .check_relay = false},
  };

  static RxCheck fca_atlantis_rx_checks[] = {
    {.msg = {{FCA_ATLANTIS_ACC_2, 2, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_ATLANTIS_ABS_1, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_ATLANTIS_ABS_3, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_ATLANTIS_ENGINE_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_ATLANTIS_EPS_2, 0, 7, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(fca_atlantis_rx_checks, FCA_ATLANTIS_TX_MSGS);
}

static uint8_t fca_atlantis_get_counter(const CANPacket_t *msg) {
  uint8_t counter;
  if (msg->addr == FCA_ATLANTIS_ABS_3) {
    counter = (uint8_t)((msg->data[4] >> 3) & 0xFU);
  } else {
    int counter_byte = GET_LEN(msg) - 2U;
    counter = (uint8_t)msg->data[counter_byte] & 0xFU;
  }
  return counter;
}

static void fca_atlantis_rx_hook(const CANPacket_t *msg) {
  // Update in-motion state by sampling wheel speeds
  if (msg_matches(msg, FCA_ATLANTIS_ABS_1, 0U)) {
    // Signals: ABS_1.WHEEL_SPEED_[FL,FR,RL,RR]
    int wheel_speed_fl = (msg->data[1] >> 3) | (msg->data[0] << 5);
    int wheel_speed_fr = (msg->data[3] >> 6) | (msg->data[2] << 2) | ((msg->data[1] & 0x7U) << 10);
    int wheel_speed_rl = (msg->data[4] >> 1) | ((msg->data[3] & 0x3FU) << 7);
    int wheel_speed_rr = (msg->data[6] >> 4) | (msg->data[5] << 4) | ((msg->data[4] & 0x1U) << 12);
    vehicle_moving = (wheel_speed_fl + wheel_speed_fr + wheel_speed_rl + wheel_speed_rr) > 0;
  }

  // Update driver input torque samples
  // Signal: EPS_2.DRIVER_TORQUE
  if (msg_matches(msg, FCA_ATLANTIS_EPS_2, 0U)) {
    int torque_driver_new = ((msg->data[2] << 3) | (msg->data[3] >> 5)) - 1024U;
    update_sample(&torque_driver, torque_driver_new);
  }

  // Signal: ENGINE_2.ACCEL_PEDAL_USER
  if (msg_matches(msg, FCA_ATLANTIS_ENGINE_2, 0U)) {
    gas_pressed = msg->data[2] > 0U;
  }

  // Signal: ABS_3.BRAKE_PEDAL_SWITCH
  if (msg_matches(msg, FCA_ATLANTIS_ABS_3, 0U)) {
    brake_pressed = GET_BIT(msg, 3U);
  }

  if (msg_matches(msg, FCA_ATLANTIS_ACC_2, 2U)) {
    // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
    // Signal: ACC_2.CRUISE_STATUS
    bool cruise_engaged = GET_BIT(msg, 23U);
    pcm_cruise_check(cruise_engaged);
  }
}

// Winddown torque limit at time ts: falls from the last torque allowed while engaged, and reaches zero by itself
static int fca_atlantis_winddown_limit(uint32_t ts) {
  uint32_t frames = safety_get_ts_elapsed(ts, fca_atlantis_ts_engaged) / FCA_ATLANTIS_FRAME_US;
  int engaged = SAFETY_ABS(fca_atlantis_engaged_torque);
  int limit = engaged - (FCA_ATLANTIS_MAX_RATE_DOWN * ((int)frames - FCA_ATLANTIS_WINDDOWN_SLACK_FRAMES));
  return SAFETY_MAX(SAFETY_MIN(limit, engaged), 0);
}

static bool fca_atlantis_winddown_allowed(int desired_torque, bool steer_req, uint32_t ts) {
  int last = fca_atlantis_winddown_torque;
  bool allowed = !controls_allowed && steer_req && (desired_torque != 0) && (last != 0);
  // same sign, magnitude not growing, and under the falling limit
  allowed = allowed && ((desired_torque > 0) == (last > 0));
  allowed = allowed && (SAFETY_ABS(desired_torque) <= SAFETY_ABS(last));
  int limit = fca_atlantis_winddown_limit(ts);
  allowed = allowed && !safety_max_limit_check(desired_torque, limit, -limit);
  return allowed;
}

static bool fca_atlantis_tx_hook(const CANPacket_t *msg) {
  // lateral limits, in LKA_COMMAND_2 units (4x LKA_COMMAND)
  const TorqueSteeringLimits FCA_ATLANTIS_STEERING_LIMITS = {
    .max_torque = 1200,
    .max_rt_delta = 448,
    .max_rate_up = 12,
    .max_rate_down = FCA_ATLANTIS_MAX_RATE_DOWN,
    .driver_torque_allowance = 80,
    .driver_torque_multiplier = 8,
    // use TorqueMotorLimited if we can find EPS torque or some EPS limit signal?
    .type = TorqueDriverLimited,
  };

  bool tx = true;

  // Safety check for commanded steering torque
  if (msg->addr == FCA_ATLANTIS_LKA_COMMAND_2) {
    // Signal: LKA_COMMAND_2.LKA_TORQUE
    int desired_torque = ((msg->data[0] << 4) | (msg->data[1] >> 4)) - 2048U;
    // Signal: LKA_COMMAND_2.LKA_ACTIVE
    bool steer_req = GET_BIT(msg, 11U);

    uint32_t ts = microsecond_timer_get();
    bool allowed;
    if (fca_atlantis_winddown_allowed(desired_torque, steer_req, ts)) {
      allowed = true;
      fca_atlantis_winddown_torque = desired_torque;
      fca_atlantis_in_winddown = true;
    } else {
      if (controls_allowed && fca_atlantis_in_winddown) {
        // Re-engaging mid-winddown: the rate limits continue from the last winddown torque. The falling limit caps
        // it, so this can't carry torque over from a winddown that has already ended.
        int limit = fca_atlantis_winddown_limit(ts);
        int seed = SAFETY_MAX(SAFETY_MIN(fca_atlantis_winddown_torque, limit), -limit);
        desired_torque_last = seed;
        rt_torque_last = seed;
        ts_torque_check_last = ts;
      }
      fca_atlantis_in_winddown = false;

      allowed = !steer_torque_cmd_checks(desired_torque, steer_req, FCA_ATLANTIS_STEERING_LIMITS);
      if (controls_allowed) {
        // the winddown starts from the last torque allowed while engaged; none after a violation
        fca_atlantis_engaged_torque = allowed ? desired_torque : 0;
        fca_atlantis_winddown_torque = fca_atlantis_engaged_torque;
        fca_atlantis_ts_engaged = ts;
      } else {
        // an allowed frame outside the winddown (zero torque) ends it. A blocked frame (e.g. one openpilot sent
        // before it saw the brake) doesn't.
        if (allowed) {
          fca_atlantis_winddown_torque = 0;
        }
      }
    }

    // LKA_COMMAND may carry one copy of this frame. After a blocked frame, only zero torque.
    fca_atlantis_lka_copy_pending = true;
    fca_atlantis_lka_copy_torque = allowed ? desired_torque : 0;
    fca_atlantis_lka_copy_steer_req = allowed && steer_req;
    if (!allowed) {
      tx = false;
    }
  }

  // LKA_COMMAND is a 1/4 scale copy of LKA_COMMAND_2. The EPS is suspected to listen only to LKA_COMMAND_2: it
  // engaged and steered with LKA_COMMAND_2 alone, and stock sends LKA_COMMAND one frame later as a rounded copy
  // of LKA_COMMAND_2 / 4. Rather than running the shared torque rate-limit state twice per frame, allow one
  // rounded copy (either direction) of each LKA_COMMAND_2, which openpilot sends first each frame.
  if (msg->addr == FCA_ATLANTIS_LKA_COMMAND) {
    // Signal: LKA_COMMAND.LKA_TORQUE
    int desired_torque = ((msg->data[0] << 3) | (msg->data[1] >> 5)) - 1024U;
    // Signal: LKA_COMMAND.LKA_ACTIVE
    bool steer_req = GET_BIT(msg, 12U);

    int rounding_error = (desired_torque * 4) - fca_atlantis_lka_copy_torque;
    bool is_copy = fca_atlantis_lka_copy_pending && !safety_max_limit_check(rounding_error, 2, -2) &&
                   (steer_req == fca_atlantis_lka_copy_steer_req);
    if (is_copy) {
      fca_atlantis_lka_copy_pending = false;
    } else {
      tx = false;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  if (msg->addr == FCA_ATLANTIS_ACC_BUTTON) {
    // Signal: ACC_BUTTON.CANCEL, with no other button and the constant bits as sent by the car
    const bool is_cancel = (msg->data[0] == 0x80U) && ((msg->data[1] & 0xF0U) == 0x30U);
    if (!is_cancel) {
      tx = false;
    }
  }

  return tx;
}

const safety_hooks fca_atlantis_hooks = {
  .init = fca_atlantis_init,
  .rx = fca_atlantis_rx_hook,
  .tx = fca_atlantis_tx_hook,
  .get_counter = fca_atlantis_get_counter,
  .get_checksum = chrysler_get_checksum,
  .compute_checksum = chrysler_compute_checksum,
};
