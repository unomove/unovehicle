from opendbc.car import get_safety_config, structs
from opendbc.car.ecar.carcontroller import CarController
from opendbc.car.ecar.carstate import CarState
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.ecar.values import EcarSafetyFlags, CarControllerParams
import numpy as np

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
      ACCEL_MAX_VALS = [CarControllerParams.ECAR_ACCEL_MAX, 2.0]
      ACCEL_MAX_BP = [cruise_speed - 2., cruise_speed - .2]
      return CarControllerParams.ECAR_ACCEL_MIN, np.interp(current_speed, ACCEL_MAX_BP, ACCEL_MAX_VALS)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.notCar = False
    ret.brand = "ecar"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.ecar)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    ret.pcmCruise = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= EcarSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    # PID varables

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kf = 6e-05
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0, 10], [0, 10]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.05, 0.5], [0.0125, 0.125]]
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0, 2560], [0, 2560]]
    ret.steerActuatorDelay = 0.1


    ret.dashcamOnly = False # candidate in (CAR.ECAR_E70) # dashcam only, pending find invalidLkasSetting signal

    return ret

# print (CarInterface.get_pid_accel_limits(None, 10, 20))