from opendbc.can import CANParser, CANDefine
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.ecar.values import DBC, CANBUS, GEAR_MAP
from opendbc.car.common.conversions import Conversions as CV


class CarState(CarStateBase):
  def __init__(self, CP: structs.CarParams):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.main])
    self.shifter_values = self.can_define.dv["CDCU_DriveStatus"]["CDCU_MCU_GearAct"] # Gear Mode
    self.cruise_enabled = True
    self.cruise_enabled_prev = False

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["CDCU_VehDyncState"]["CDCU_Veh_LongtdnalSpd"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    ret.gasPressed = cp.vl["CDCU_DriveStatus"]["CDCU_MCU_ThrotAct"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp.vl["CDCU_BrakeStatus"]["CDCU_EHB_BrkPedpos"] > 0

    steer_status = cp.vl["CDCU_SteerStatus"]
    ret.steeringAngleDeg = -steer_status["CDCU_EPS_StrWhlAngle"]
    ret.steeringRateDeg = -steer_status["CDCU_EPS_WhlSpd"]
    ret.steeringTorque = -steer_status["CDCU_EPS_StrTrq"]
    ret.steeringPressed = True
    ret.steerFaultPermanent = steer_status["CDCU_EPS_ErrLevel"] > 0x0
    # print ("steer value", ret.steeringAngleDeg, ret.steeringRateDeg, ret.steeringTorque, steer_status["CDCU_EPS_WorkMode"], ret.steerFaultPermanent)
    # print ("speed value", ret.vEgoRaw)
    # ret.steeringAngleDeg =0
    # ret.steeringRateDeg = 0
    # ret.steeringTorque = 0

    # ret.steerFaultPermanent =False

    # vehicle finite state machine
    # ret.run_mode = cp.vl["CDCU_VehState"]["CDCU_Veh_RunMode"]
    # irrelevant for non-car
    ret.gearShifter = GEAR_MAP[self.shifter_values.get(int(cp.vl["CDCU_DriveStatus"]["CDCU_MCU_GearAct"]))]
    ret.cruiseState.speed = 20 * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = 20 * CV.KPH_TO_MS
    ret.cruiseState.enabled = True
    ret.cruiseState.available = True

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], [], CANBUS.vehicle)}
