#pragma once

#include "opendbc/safety/safety_declarations.h"

// Safety-relevant CAN messages for EUCD platform.
#define VOLVO_EUCD_AccPedal      0x020  // RX, gas pedal
#define VOLVO_EUCD_FSM0          0x051  // RX from FSM, cruise state
#define VOLVO_EUCD_VehicleSpeed1 0x148  // RX, vehicle speed
#define VOLVO_EUCD_Brake_Info    0x20a  // RX, driver brake pressed
#define VOLVO_EUCD_CCButtons     0x127  // TX by OP, CC buttons
#define VOLVO_EUCD_PSCM1         0x246  // TX by OP to camera, PSCM state
#define VOLVO_EUCD_FSM2          0x262  // TX by OP, LKA command
#define VOLVO_EUCD_FSM3          0x270  // TX by OP, ACC status

// CAN bus numbers.
#define VOLVO_MAIN_BUS 0U
#define VOLVO_AUX_BUS  1U
#define VOLVO_CAM_BUS  2U

static const CanMsg VOLVO_EUCD_TX_MSGS[] = {
    {VOLVO_EUCD_CCButtons, VOLVO_MAIN_BUS, 8, .check_relay = false},
    {VOLVO_EUCD_PSCM1,     VOLVO_CAM_BUS,  8, .check_relay = true},
    {VOLVO_EUCD_FSM2,      VOLVO_MAIN_BUS, 8, .check_relay = true},
    {VOLVO_EUCD_FSM3,      VOLVO_MAIN_BUS, 8, .check_relay = false}
  };

  // TODO: add counters
  static RxCheck volvo_eucd_rx_checks[] = {
    {.msg = {{VOLVO_EUCD_AccPedal,      VOLVO_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_FSM0,          VOLVO_CAM_BUS,  8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_VehicleSpeed1, VOLVO_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_Brake_Info,    VOLVO_MAIN_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},
  };

static void volvo_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == VOLVO_MAIN_BUS) {
    if (addr == VOLVO_EUCD_VehicleSpeed1) {
      // Signal: VehicleSpeed
      unsigned int speed_raw = (GET_BYTE(to_push, 6) << 8) | GET_BYTE(to_push, 7);
      vehicle_moving = speed_raw >= 36U;
      UPDATE_VEHICLE_SPEED(speed_raw * 0.01 / 3.6);
    }

    if (addr == VOLVO_EUCD_AccPedal) {
      // Signal: AccPedal
      unsigned int gas_raw = ((GET_BYTE(to_push, 2) & 0x03U) << 8) | GET_BYTE(to_push, 3);
      gas_pressed = gas_raw >= 100U;
    }

    if (addr == VOLVO_EUCD_Brake_Info) {
      // Signal: BrakePedal
      brake_pressed = ((GET_BYTE(to_push, 2) & 0x0CU) >> 2U) == 2U;
    }

    // If steering controls messages are received on the destination bus, it's an indication
    // that the relay might be malfunctioning.
    // generic_rx_checks(volvo_lkas_msg_check(addr));
  } else if (bus == VOLVO_CAM_BUS) {
    if (addr == VOLVO_EUCD_FSM0) {
      // Signal: ACCStatus
      unsigned int cruise_state = GET_BYTE(to_push, 2) & 0x07U;
      bool cruise_engaged = (cruise_state == 6U) || (cruise_state == 7U);
      pcm_cruise_check(cruise_engaged);
    }
  }
}

static bool volvo_tx_hook(const CANPacket_t *to_send) {
  //const AngleSteeringLimits VOLVO_STEERING_LIMITS = {
  //  .max_angle = 60000,  // 600 deg, reasonable limit
  //  .angle_deg_to_can = 100,
  //  .angle_rate_up_lookup = {
  //    {0., 5., 15.},
  //    {5., .8, .15}
  //  },
  //  .angle_rate_down_lookup = {
  //    {0., 5., 15.},
  //    {5., 3.5, .4}
  //  },
  //};

  bool tx = true;
  int addr = GET_ADDR(to_send);
  bool violation = false;

  // Safety check for CC button signals.
  if (addr == VOLVO_EUCD_CCButtons) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    violation |= !cruise_engaged_prev && (GET_BIT(to_send, 59U) || !(GET_BIT(to_send, 43U)));  // Signals: ACCOnOffBtn, ACCOnOffBtnInv (cancel)
    violation |= !controls_allowed && (GET_BIT(to_send, 61U) || !(GET_BIT(to_send, 45U)));  // Signals: ACCResumeBtn, ACCResumeBtnInv (resume)
  }

  // Safety check for Lane Keep Assist action.
  if (addr == VOLVO_EUCD_FSM2) {
    // Signal: LKASteerDirection
    unsigned int mode = GET_BYTE(to_send, 5) & 0x03U;
    bool lka_active = mode != 0U;

    if (lka_active && !controls_allowed) {
      violation = true;
    }
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static safety_config volvo_init(uint16_t param) {
  
  UNUSED(param);

  return BUILD_SAFETY_CFG(volvo_eucd_rx_checks, VOLVO_EUCD_TX_MSGS);
}

const safety_hooks volvo_hooks = {
  .init = volvo_init,
  .rx = volvo_rx_hook,
  .tx = volvo_tx_hook,
};
