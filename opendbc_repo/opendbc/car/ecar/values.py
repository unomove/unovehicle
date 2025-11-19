from dataclasses import dataclass, field
from opendbc.car import DbcDict, Bus, CarSpecs, PlatformConfig, Platforms, ACCELERATION_DUE_TO_GRAVITY
from opendbc.car.structs import CarParams, CarState
from opendbc.car.docs_definitions import CarDocs, CarFootnote, Column, CarParts, CarHarness
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL
from enum import Enum, IntFlag
from opendbc.car.common.conversions import Conversions as CV

Ecu = CarParams.Ecu

class Footnote(Enum):
  HW_TYPE = CarFootnote(
    "UnoBox can use different chips, i.e. jetson orin, AMD, Intel etc.",
    Column.MODEL
  )

  SETUP = CarFootnote(
    "Setup details is under construction...",
    Column.MAKE, setup_note=True
  )

@dataclass
class EcarCarDocsE70(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.ecar_jetson]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.HW_TYPE, Footnote.SETUP])

@dataclass
class EcarPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.main: 'ecar_e70'})


class CAR(Platforms):
  # ECAR_E70 = EcarPlatformConfig(
  #   [
  #     EcarCarDocsE70("Ecar E70 with Jetson Orin"),
  #   ],
  #   CarSpecs(mass=260., wheelbase=900, steerRatio=1.0), #steerRatio is 1 here, since we directly control wheel steer angle
  # ),
  ECAR_E70 = EcarPlatformConfig(
    [
      EcarCarDocsE70("Ecar E70 with Jetson Orin"),
    ],
    CarSpecs(mass=1326, wheelbase=2.7, steerRatio=15.38, centerToFrontRatio=0.4)
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)


# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration
class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # EPAS faults above this angle
    360,  # deg
    # Tesla uses a vehicle model instead, check carcontroller.py for details
    ([], []),
    ([], []),

    # Vehicle model angle limits
    # Add extra tolerance for average banked road since safety doesn't have the roll
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^2
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),  # ~3.6 m/s^3

    # limit angle rate to both prevent a fault and for low speed comfort (~12 mph rate down to 0 mph)
    MAX_ANGLE_RATE=5,  # deg/20ms frame, EPS faults at 12 at a standstill
  )

  SPEED_MAX = 5 #km/h, max_speed of Ecar E70
  STEER_ANGLE_MAX = 30 # deg, max steering angle of Ecar E70

  ECAR_ACCEL_MIN = -4.0  # m/s^2
  ECAR_ACCEL_MAX = 1.6  # m/s^2, lower than 2.0 m/s^2 for tuning reasons

  ECAR_ACCEL_LOOKUP_BP = [-1., 0., .6]
  ECAR_ACCEL_LOOKUP_V = [-4.8, 0., 2.0]

  ECAR_MAX_ACCEL_V = [0.5, 2.4, 1.4, 0.6]
  ECAR_MAX_ACCEL_BP = [0.0, 4.0, 10., 20.]

  ECAR_GAS_MAX = 198  # 0xc6
  ECAR_BRAKE_MAX = 1024 // 4

  ECAR_GAS_LOOKUP_BP = [-0.2, 2.0]  # 2m/s^2
  ECAR_GAS_LOOKUP_V = [0, 1600]

  STEER_STEP = 1  # 100 Hz
  STEER_DELTA_UP = 3  # min/max in 0.33s for all Honda
  STEER_DELTA_DOWN = 3
  STEER_GLOBAL_MIN_SPEED = 3 * CV.MPH_TO_MS

  def __init__(self, CP):
    self.STEER_MAX = CP.lateralParams.torqueBP[-1]
    # mirror of list (assuming first item is zero) for interp of signed request
    # values and verify that both arrays begin at zero
    assert CP.lateralParams.torqueBP[0] == 0
    assert CP.lateralParams.torqueV[0] == 0
    self.STEER_LOOKUP_BP = [v * -1 for v in CP.lateralParams.torqueBP][1:][::-1] + list(CP.lateralParams.torqueBP)
    self.STEER_LOOKUP_V = [v * -1 for v in CP.lateralParams.torqueV][1:][::-1] + list(CP.lateralParams.torqueV)

class CANBUS:
  vehicle = 0

class GEAR:
  #0: Neutral, 1: Drive, 2: Reverse,
  N = 0x0
  D = 0x1
  R = 0x2
  RESERVE = 0x3

class RunDirection:
  STOP = 0
  FORWARD = 1
  BACKWORD = 2
  RESERVE = 3

class RunMode:
  STANDBY = 0
  INVALID = 1
  REMOTE = 2
  AUTOMATIC = 3
  EMERGENCY = 4
  CLOUD = 5

GEAR_INVERSE_MAP = {
  "unknown": GEAR.RESERVE,
  "neutral": GEAR.N,
  "drive": GEAR.D,
  "reverse": GEAR.R
}


GEAR_MAP = {
  "RESERVE": CarState.GearShifter.unknown,
  "R": CarState.GearShifter.reverse,
  "N": CarState.GearShifter.neutral,
  "D": CarState.GearShifter.drive,
}

class EcarSafetyFlags(IntFlag):
  LONG_CONTROL = 1


DBC = CAR.create_dbc_map()

