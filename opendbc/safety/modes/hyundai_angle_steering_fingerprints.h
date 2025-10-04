#pragma once
#include "opendbc/safety/modes/hyundai_uds_handler_declarations.h"

// HYUNDAI_SANTA_FE_HEV_5TH_GEN values. (values can be found on values.py)
extern const AngleSteeringParams HYUNDAI_SANTA_FE_HEV_5TH_GEN_STEERING_PARAMS;
const AngleSteeringParams HYUNDAI_SANTA_FE_HEV_5TH_GEN_STEERING_PARAMS = {
  .slip_factor = -0.00059689759884299,  // calc_slip_factor(VM)
  .steer_ratio = 13.72,
  .wheelbase = 2.81,
};

  // IONIQ 5 PE values.
extern const AngleSteeringParams HYUNDAI_IONIQ_5_PE_STEERING_PARAMS;
const AngleSteeringParams HYUNDAI_IONIQ_5_PE_STEERING_PARAMS = {
    .slip_factor = -0.0008688329819908074,  // calc_slip_factor(VM)
    .steer_ratio = 14.26,
    .wheelbase = 2.97,
  };

// GENESIS_GV80_2025 values. (values can be found on values.py)
extern const AngleSteeringParams GENESIS_GV80_2025_STEERING_PARAMS;
const AngleSteeringParams GENESIS_GV80_2025_STEERING_PARAMS = {
  .slip_factor = -0.0005685702046115589,  // calc_slip_factor(VM)
  .steer_ratio = 14.14,
  .wheelbase = 2.95,
};

// // KIA EV9 values. (values can be found on values.py)
extern const AngleSteeringParams KIA_EV9_STEERING_PARAMS;
const AngleSteeringParams KIA_EV9_STEERING_PARAMS = {
  .slip_factor = -0.0005410588125765342,  // calc_slip_factor(VM)
  .steer_ratio = 16,
  .wheelbase = 3.1,
};

// KIA_SPORTAGE_HEV_2026 values. (most conservative for now) (values can be found on values.py)
extern const AngleSteeringParams KIA_SPORTAGE_HEV_2026_STEERING_PARAMS;
const AngleSteeringParams KIA_SPORTAGE_HEV_2026_STEERING_PARAMS = {
  .slip_factor = -0.0006085930193026732,  // calc_slip_factor(VM)
  .steer_ratio = 13.7,
  .wheelbase = 2.756,
};

#define HKG_FALLBACK_STEERING_PARAMS KIA_SPORTAGE_HEV_2026_STEERING_PARAMS

static const hkg_angle_fingerprint_t HKG_ECU_STEERING_FINGERPRINTS[] = {
  {HKG_CAM_UDS_ADDR, "NE  MFC  AT USA LHD 1.00 1.01 99211-PI000 240905", &HYUNDAI_IONIQ_5_PE_STEERING_PARAMS},
  {HKG_CAM_UDS_ADDR, "NE  MFC  AT EUR LHD 1.00 1.03 99211-GI500 240809", &HYUNDAI_IONIQ_5_PE_STEERING_PARAMS},
  {HKG_CAM_UDS_ADDR, "JX  MFC  AT USA LHD 1.00 1.03 99211-T6510 240124", &GENESIS_GV80_2025_STEERING_PARAMS},
  {HKG_CAM_UDS_ADDR, "MX5HMFC  AT KOR LHD 1.00 1.07 99211-P6000 231218", &HYUNDAI_SANTA_FE_HEV_5TH_GEN_STEERING_PARAMS},
  {HKG_CAM_UDS_ADDR, "MX5HMFC  AT USA LHD 1.00 1.06 99211-R6000 231218", &HYUNDAI_SANTA_FE_HEV_5TH_GEN_STEERING_PARAMS},
  {HKG_CAM_UDS_ADDR, "NQ51.011.021.012551000HKP_NQ524_50509099211P1110", &KIA_SPORTAGE_HEV_2026_STEERING_PARAMS},
  {HKG_CAM_UDS_ADDR, "MV__ RDR -----      1.00 1.02 99110-DO000         ", &KIA_EV9_STEERING_PARAMS},
  {HKG_CAM_UDS_ADDR, "MV__ RDR -----      1.00 1.03 99110-DO000         ", &KIA_EV9_STEERING_PARAMS},
  {HKG_CAM_UDS_ADDR, "MV__ RDR -----      1.00 1.04 99110-DO000         ", &KIA_EV9_STEERING_PARAMS},
  {HKG_CAM_UDS_ADDR, "MV__ RDR -----      1.00 1.02 99110-DO700         ", &KIA_EV9_STEERING_PARAMS},
};

static bool HKG_ECU_VERSION_MATCH(const char *a, const char *b) {
  const char *ptr_a = a;
  const char *ptr_b = b;
  
  while (*ptr_a && *ptr_b && *ptr_a == *ptr_b) {
    ptr_a++;
    ptr_b++;
  }
  return *ptr_a == *ptr_b;
}

static const AngleSteeringParams *HYUNDAI_LOOKUP_STEERING_PARAMS(uint32_t ecu_address, const hkg_uds_data_t *uds_data) {
  const AngleSteeringParams *result = &HKG_FALLBACK_STEERING_PARAMS;
  for (size_t i = 0; i < ARRAY_SIZE(HKG_ECU_STEERING_FINGERPRINTS); i++) {
    const hkg_angle_fingerprint_t *ecu_fingerprint = &HKG_ECU_STEERING_FINGERPRINTS[i];
    if (ecu_address == ecu_fingerprint->ecu_address && HKG_ECU_VERSION_MATCH(uds_data->ecu_software_version, ecu_fingerprint->ecu_software_version)) {
      result = ecu_fingerprint->steering_params;
      break;
    }
  }
  return result;
}
