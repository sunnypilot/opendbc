from dataclasses import dataclass, field
from enum import Enum, IntFlag
import re

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimitsVM
from opendbc.car.structs import CarParams, CarState
from opendbc.car.docs_definitions import CarDocs, CarFootnote, CarHarness, CarParts, Column, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class Footnote(Enum):
  HW_TYPE = CarFootnote(
    "Some 2023 model years have HW4. To check which hardware type your vehicle has, look for " +
    "<b>Autopilot computer</b> under <b>Software -> Additional Vehicle Information</b> on your vehicle's touchscreen. </br></br>" +
    "See <a href=\"https://www.notateslaapp.com/news/2173/how-to-check-if-your-tesla-has-hardware-4-ai4-or-hardware-3\">this page</a> for more information.",
    Column.MODEL)

  SETUP = CarFootnote(
    "See more setup details for <a href=\"https://github.com/commaai/openpilot/wiki/tesla\" target=\"_blank\">Tesla</a>.",
    Column.MAKE, setup_note=True)


@dataclass
class TeslaCarDocsHW3(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_a]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])


@dataclass
class TeslaCarDocsHW4(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.tesla_b]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])

@dataclass
class TeslaCarHW4ModelSXDocs(TeslaCarDocsHW4):
  support_type: SupportType = SupportType.COMMUNITY
  support_link: str = "community"


@dataclass
class TeslaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.party: 'tesla_model3_party', Bus.adas: 'tesla_model3_vehicle'})


class CAR(Platforms):
  TESLA_MODEL_3 = TeslaPlatformConfig(
    [
      # TODO: do we support 2017? It's HW3
      TeslaCarDocsHW3("Tesla Model 3 (with HW3) 2019-23"),
      TeslaCarDocsHW4("Tesla Model 3 (with HW4) 2024-25"),
    ],
    CarSpecs(mass=1899., wheelbase=2.875, steerRatio=12.0),
    {Bus.party: 'tesla_model3_party', Bus.radar: 'tesla_radar_continental_generated', Bus.adas: 'tesla_model3_vehicle'},
  )
  TESLA_MODEL_Y = TeslaPlatformConfig(
    [
      TeslaCarDocsHW3("Tesla Model Y (with HW3) 2020-23"),
      TeslaCarDocsHW4("Tesla Model Y (with HW4) 2024-25"),
    ],
    CarSpecs(mass=2072., wheelbase=2.890, steerRatio=12.0),
    {Bus.party: 'tesla_model3_party', Bus.radar: 'tesla_radar_continental_generated', Bus.adas: 'tesla_model3_vehicle'},
  )
  TESLA_MODEL_X = TeslaPlatformConfig(
    [TeslaCarHW4ModelSXDocs("Tesla Model X (with HW4) 2024")],
    CarSpecs(mass=2495., wheelbase=2.960, steerRatio=12.0),
  )


TESLA_EPS_FW_RE = re.compile(
  rb'^.+,(?P<platform>[EYX])(?P<variant_code>\d?[A-Z]*\d{3})\.(?P<software_major>\d+)(?:\.\d+)?$'
)


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  """Identify an otherwise unknown Tesla EPS FW by its embedded platform code.

  Tesla's EPS response is the only queried ECU and E/Y/X is the only portion
  whose meaning is established. Requiring address 0x730 and exactly one valid
  platform keeps this fallback narrower than a generic CAN fingerprint match.
  """
  del vin, offline_fw_versions
  platforms = set()
  for (address, sub_address), versions in live_fw_versions.items():
    if address != 0x730 or sub_address is not None:
      continue
    for version in versions:
      match = TESLA_EPS_FW_RE.fullmatch(version)
      if match is not None:
        platforms.add(match["platform"])

  # This fallback is intentionally limited to Model Y HW4/Juniper. Extend it
  # to E/X only after those refresh platforms have equivalent validation.
  return {str(CAR.TESLA_MODEL_Y)} if platforms == {b"Y"} else set()


def is_fsd_14_fw(candidate, fw_version: bytes) -> bool:
  match = TESLA_EPS_FW_RE.fullmatch(fw_version)
  if match is None or int(match["software_major"]) < 4:
    return False

  variant = match["variant_code"]
  if candidate == CAR.TESLA_MODEL_3:
    return variant.startswith(b"4H") and variant.endswith(b"015")
  if candidate == CAR.TESLA_MODEL_Y:
    return variant.startswith(b"4") and variant.endswith(b"003")
  return False


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br".+,[EYX]\d?[A-Z]*\d{3}\.\d+(?:\.\d+)?",
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    )
  ],
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

# Cars with this EPS FW have FSD 14 and use TeslaFlags.FSD_14
FSD_14_FW = {
  CAR.TESLA_MODEL_3: [
    b'TeMYG4_Main_0.0.0 (77),E4HP015.04.5',
    b'TeMYG4_Main_0.0.0 (78),E4HP015.05.0',
    b'TeMYG4_Main_0.0.0 (77),E4H015.04.5',
    b'TeMYG4_Main_0.0.0 (78),E4H015.05.0',
  ],
  CAR.TESLA_MODEL_Y: [
    b'TeMYG4_Legacy3Y_0.0.0 (6),Y4003.04.0',
    b'TeMYG4_Main_0.0.0 (77),Y4003.05.4',
    b'TeMYG4_Main_0.0.0 (78),Y4003.06.0',
  ]
}


class CANBUS:
  party = 0
  vehicle = 1
  autopilot_party = 2


GEAR_MAP = {
  "DI_GEAR_INVALID": CarState.GearShifter.unknown,
  "DI_GEAR_P": CarState.GearShifter.park,
  "DI_GEAR_R": CarState.GearShifter.reverse,
  "DI_GEAR_N": CarState.GearShifter.neutral,
  "DI_GEAR_D": CarState.GearShifter.drive,
  "DI_GEAR_SNA": CarState.GearShifter.unknown,
}


class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    # EPAS faults above this angle
    360,  # deg
    # limit angle rate to both prevent a fault and for low speed comfort (~12 mph rate down to 0 mph)
    MAX_ANGLE_RATE=5,  # deg/20ms frame, EPS faults at 12 at a standstill
  )

  STEER_STEP = 2  # Angle command is sent at 50 Hz
  ACCEL_MAX = 2.0    # m/s^2
  ACCEL_MIN = -3.48  # m/s^2
  JERK_LIMIT_MAX = 4.9  # m/s^3, ACC faults at 5.0
  JERK_LIMIT_MIN = -4.9  # m/s^3, ACC faults at 5.0


class TeslaSafetyFlags(IntFlag):
  LONG_CONTROL = 1
  FSD_14 = 2


class TeslaFlags(IntFlag):
  LONG_CONTROL = 1
  FSD_14 = 2
  MISSING_DAS_SETTINGS = 4


DBC = CAR.create_dbc_map()

STEER_THRESHOLD = 1
