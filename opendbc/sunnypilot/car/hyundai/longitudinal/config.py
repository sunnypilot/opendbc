"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass
from enum import StrEnum

from opendbc.car.hyundai.values import CAR


class SCC14Family(StrEnum):
  RATE = "rate"  # jerk fields follow aReqValue's rate, comfort bands with a lead
  FIXED = "fixed"  # constant jerk fields, no bands
  ZEROS = "zeros"  # fields and bands 0, aReqValue is the request
  NONE = "none"  # no SCC14, aReqValue is the request
  STEPPED = "stepped"  # CAN-FD: one level per field while accelerating, another while braking


@dataclass
class CarTuningConfig:
  longitudinal_actuator_delay: float = 0.50
  jerk_upper_floor: float = 0.3
  jerk_lower_floor: tuple[float, float] | None = None  # without, with a lead; None is 0.5, 1.0
  scc14: SCC14Family = SCC14Family.RATE
  scc14_jerk: tuple[float, float] = (0.0, 0.0)  # FIXED: upper, lower
  stepped_jerk: tuple[float, float, float, float] = (0.7, 1.8, 1.6, 1.0)  # STEPPED: upper, lower accelerating; upper, lower braking
  canfd_band: tuple[float, float] = (0.0, 0.0)  # CAN-FD: upper, lower with a lead


# Default configurations for different car types
TUNING_CONFIGS = {
  "CANFD": CarTuningConfig(),
  "EV": CarTuningConfig(),
  "HYBRID": CarTuningConfig(),
  "DEFAULT": CarTuningConfig(),
}

# what each car's own radar sends on stock cruise
CAR_SPECIFIC_CONFIGS: dict[str, CarTuningConfig] = {
  CAR.HYUNDAI_SONATA: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.HYUNDAI_SONATA_HYBRID: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.KIA_K5_2021: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.KIA_STINGER_2022: CarTuningConfig(jerk_upper_floor=0.5, jerk_lower_floor=(0.5, 2.5)),
  CAR.KIA_SELTOS: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.HYUNDAI_NEXO_1ST_GEN: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.HYUNDAI_ELANTRA_2021: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.HYUNDAI_ELANTRA_HEV_2021: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.HYUNDAI_SANTA_FE: CarTuningConfig(jerk_upper_floor=0.5, jerk_lower_floor=(1.0, 1.0)),
  CAR.HYUNDAI_SANTA_FE_2022: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.HYUNDAI_SANTA_FE_HEV_2022: CarTuningConfig(jerk_upper_floor=0.5, jerk_lower_floor=(0.5, 0.5)),
  CAR.HYUNDAI_SANTA_FE_PHEV_2022: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.HYUNDAI_PALISADE: CarTuningConfig(jerk_upper_floor=1.0, jerk_lower_floor=(0.5, 0.5)),
  CAR.HYUNDAI_KONA_2022: CarTuningConfig(jerk_upper_floor=0.5, jerk_lower_floor=(0.5, 2.0)),
  CAR.HYUNDAI_KONA_EV_2022: CarTuningConfig(jerk_upper_floor=1.0, jerk_lower_floor=(0.5, 2.0)),
  CAR.HYUNDAI_CUSTIN_1ST_GEN: CarTuningConfig(scc14=SCC14Family.FIXED, scc14_jerk=(1.0, 0.5)),
  CAR.HYUNDAI_IONIQ_EV_2020: CarTuningConfig(scc14=SCC14Family.ZEROS),
  CAR.HYUNDAI_IONIQ_HEV_2022: CarTuningConfig(scc14=SCC14Family.ZEROS),
  CAR.HYUNDAI_IONIQ_PHEV: CarTuningConfig(scc14=SCC14Family.ZEROS),
  CAR.HYUNDAI_AZERA_6TH_GEN: CarTuningConfig(scc14=SCC14Family.ZEROS),
  CAR.KIA_NIRO_HEV_2021: CarTuningConfig(scc14=SCC14Family.ZEROS),
  CAR.KIA_NIRO_PHEV: CarTuningConfig(scc14=SCC14Family.ZEROS),
  CAR.KIA_NIRO_PHEV_2022: CarTuningConfig(scc14=SCC14Family.ZEROS),
  CAR.GENESIS_G90: CarTuningConfig(scc14=SCC14Family.ZEROS),
  CAR.GENESIS_G70: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.GENESIS_G80: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.HYUNDAI_GENESIS: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.HYUNDAI_TUCSON: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.HYUNDAI_SONATA_LF: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.HYUNDAI_IONIQ: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.HYUNDAI_IONIQ_PHEV_2019: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.HYUNDAI_IONIQ_EV_LTD: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.HYUNDAI_AZERA_HEV_6TH_GEN: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.KIA_OPTIMA_G4: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.KIA_OPTIMA_G4_FL: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.KIA_OPTIMA_H_G4_FL: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.KIA_SORENTO: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.KIA_STINGER: CarTuningConfig(scc14=SCC14Family.NONE),
  CAR.HYUNDAI_STARIA_4TH_GEN: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.KIA_EV6: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(0.5, 2.0, 0.5, 0.6)),
  CAR.KIA_CARNIVAL_4TH_GEN: CarTuningConfig(jerk_upper_floor=0.5, jerk_lower_floor=(0.5, 1.5)),
  CAR.KIA_SORENTO_4TH_GEN: CarTuningConfig(jerk_upper_floor=0.5),
  CAR.KIA_SORENTO_HEV_4TH_GEN: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(0.5, 0.9, 0.1, 1.0), canfd_band=(0.0, 0.02)),
  CAR.KIA_NIRO_HEV_2ND_GEN: CarTuningConfig(jerk_upper_floor=0.5, jerk_lower_floor=(0.5, 1.2), canfd_band=(0.08, 0.04)),
  CAR.GENESIS_GV60_EV_1ST_GEN: CarTuningConfig(jerk_upper_floor=0.5, jerk_lower_floor=(0.5, 1.5), canfd_band=(0.08, 0.04)),
  CAR.GENESIS_GV80: CarTuningConfig(jerk_upper_floor=0.2),
  CAR.HYUNDAI_IONIQ_5: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(0.5, 1.0, 0.5, 1.1)),
  CAR.HYUNDAI_IONIQ_6: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(0.5, 2.0, 1.7, 1.0)),
  CAR.KIA_NIRO_EV_2ND_GEN: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(0.5, 1.6, 2.5, 1.1)),
  CAR.GENESIS_GV70_1ST_GEN: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(0.6, 1.9, 0.6, 1.9), canfd_band=(0.0, 0.08)),
  CAR.HYUNDAI_TUCSON_4TH_GEN: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(0.6, 1.7, 1.2, 1.1), canfd_band=(0.1, 0.1)),
  CAR.KIA_SPORTAGE_5TH_GEN: CarTuningConfig(jerk_upper_floor=0.8, canfd_band=(0.1, 0.1)),
  CAR.GENESIS_G80_2ND_GEN_FL: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(0.8, 2.9, 4.3, 1.0), canfd_band=(0.2, 0.2)),
  CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(0.3, 1.7, 1.1, 1.1), canfd_band=(0.06, 0.06)),
  CAR.HYUNDAI_KONA_EV_2ND_GEN: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(1.2, 2.9, 1.7, 1.1)),
  CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN: CarTuningConfig(scc14=SCC14Family.STEPPED, stepped_jerk=(0.6, 1.3, 1.2, 1.0)),
}
