#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/defaults.h"

// Cherry (Jaecoo J7 PHEV, etc.) — TX whitelist for LANE_KEEP; cruise engagement gates controls_allowed
// via HUD on camera bus (matches cherry_general_pt.dbc + CarState cruise parsing).
#define CHERRY_LANE_KEEP   0x345U
#define CHERRY_LKAS_INFO   0x394U
#define CHERRY_HUD         0x387U
#define CHERRY_PCM_BUTTONS 0x360U

// Always on for Jaecoo J7: spoof LKAS_INFO on bus 0 and block stock 0x394 from camera bus 2.
static bool cherry_torque_spoof = true;

static RxCheck cherry_rx_checks[] = {
  {.msg = {{CHERRY_HUD, 2U, 8U, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
};

static const CanMsg CHERRY_TX_MSGS[] = {
  {CHERRY_LANE_KEEP, 0, 8, .check_relay = true},  // bus 0 = vehicle/ECU side; stock cam TXes on bus 2.
  {CHERRY_LKAS_INFO, 0, 8, .check_relay = true},  // torque spoof on PT bus (CarState LKAS_INFO).
  {CHERRY_PCM_BUTTONS, 0, 6, .check_relay = false},  // stock ACC resume (ICC_TOGGLE press).
};

static void cherry_rx_hook(const CANPacket_t *msg) {
  // HUD CRUISE_STATE on camera leg (panda bus 2): DBC CM "3 is ENABLE" — same as CarState.cruiseState.enabled.
  if ((msg->addr == CHERRY_HUD) && (msg->bus == 2U) && (GET_LEN(msg) >= 5U)) {
    const uint8_t cruise_state = (uint8_t)((msg->data[4] >> 2) & 0x3U);
    const bool cruise_engaged = (cruise_state == 3U);
    pcm_cruise_check(cruise_engaged);
  }
}

static safety_config cherry_init(uint16_t param) {
  SAFETY_UNUSED(param);
  controls_allowed = false;
  return BUILD_SAFETY_CFG(cherry_rx_checks, CHERRY_TX_MSGS);
}

static bool cherry_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return true;
}

static bool cherry_fwd_hook(int bus_num, int addr) {
  if (bus_num == 2) {
    if (addr == (int)CHERRY_LANE_KEEP) {
      return true;
    }
    if (cherry_torque_spoof && (addr == (int)CHERRY_LKAS_INFO)) {
      return true;
    }
  }
  return false;
}

const safety_hooks cherry_hooks = {
  .init = cherry_init,
  .rx = cherry_rx_hook,
  .tx = cherry_tx_hook,
  .fwd = cherry_fwd_hook,
};
