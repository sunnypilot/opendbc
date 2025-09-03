from dataclasses import dataclass, field
from enum import IntEnum

from opendbc.car import AngleSteeringLimits, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds, DT_CTRL
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16
from opendbc.car.structs import CarParams

from opendbc.can.can_define import CANDefine

Ecu = CarParams.Ecu

"""
Volvo Electronic Control Units abbreviations and network topology
Platforms C1MCA/EUCD

Three main CAN network buses
  1. Powertrain
  2. Chassis (also called MS* CAN) *MS=Medium Speed
  3. Extended
Only mentioning control units of interest on the network buses.

Powertrain CAN
  BCM - Brake Control Module
  CEM - Central Electronic Module
  CVM - Closing Velocity Module (low speed auto emergency braking <30kph)
  FSM - Forward Sensing Module (camera mounted in windscreen)
  PPM - Pedestrian Protection Module (controls pedestrian airbag under the engine hood)
  PSCM - Power Steering Control Module (EPS - Electronic Power Steering)
  SAS - Steering Angle Sensor Module
  SRS - Supplemental Restraint System Module (seatbelts, airbags...)
  TCM - Transmission Control Module

Chassis CAN
  CEM - Central Electronic Module
  DIM - Driver Information Module (the instrument cluster with odo and speedometer, relayed thru CEM)
  PAM - Parking Assistance Module (automatic parking, relayed thru CEM)

Extended CAN
  CEM - Central Electronic Module
  SODL - Side Object Detection Left (relayed thru CEM)
  SODR - Side Object Detection Right (relayed thru CEM)
"""


class SteerDirection(IntEnum):
  """Constants for LKASteerDirection.

  On the EUCD platform, we need to wait 8 frames when switching from LEFT to RIGHT
  and vice versa. On the C1MCA platform, we can instead use BOTH."""
  NONE = 0
  RIGHT = 1
  LEFT = 2
  BOTH = 3


class CarControllerParams:
  # EUCD: Torque limit for steering is 50 CAN units
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(90,  # deg, reasonable limit
    ([0., 5., 15.], [5., .8, .15]),
    ([0., 5., 15.], [5., 3.5, 0.4]),
  )

  # Temporary steer fault timeout
  # Maximum time to continuously read 0 torque from EPS
  STEER_TIMEOUT = 30 / DT_CTRL

  # EUCD
  # When changing steer direction steering request need to be blocked.
  # Otherwise servo won't "listen" to the request.
  # This calibration sets the number of samples to block steering request.
  BLOCK_LEN = 8
  # When close to desired steering angle, don't change steer direction inside deadzone.
  # Since we need to release control of the steering wheel for a brief moment, steering wheel will
  # unwind by itself.
  DEADZONE = 0.2

  def __init__(self, CP):
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    #pass

class CANBUS:
  pt = 0
  body = 1
  cam = 2

@dataclass
class VolvoEUCDPlatformConfig(PlatformConfig):
  #dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('volvo_v60_2015_pt', None))
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'volvo_v60_2015_pt'})


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control & Lane Keeping Aid"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass(frozen=True)
class VolvoCarSpecs(CarSpecs):
  steerRatio: float = 15.0
  centerToFrontRatio: float = 0.44
  minSteerSpeed: float = 1.0


class CAR(Platforms):
  #config: VolvoEUCDPlatformConfig

  VOLVO_V60 = VolvoEUCDPlatformConfig(
    [VolvoCarDocs("Volvo V60")],
    VolvoCarSpecs(mass=1750, wheelbase=2.776),
  )


VOLVO_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf1a2)
VOLVO_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf1a2)

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [VOLVO_VERSION_REQUEST],
      [VOLVO_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
