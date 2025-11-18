import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, rate_limit
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.ecar.ecarcan import EcarCAN
from opendbc.car.ecar.values import CarControllerParams
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car import structs

def get_safety_CP():
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("ECAR_E70")

def compute_gas_brake(accel, speed):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(accel) / 4.8 - creep_brake
  return np.clip(gb, 0.0, 1.0), np.clip(-gb, 0.0, 1.0)

  # TODO not clear this does anything useful
def actuator_hysteresis(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params
  brake_hyst_on = 0.02    # to activate brakes exceed this value
  brake_hyst_off = 0.005  # to deactivate brakes below this value
  brake_hyst_gap = 0.01   # don't change brake command for small oscillations within this value

  # *** hysteresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  return brake, braking, brake_steady

def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_ts, ts):
  pump_on = False

  # reset pump timer if:
  # - there is an increment in brake request
  # - we are applying steady state brakes and we haven't been running the pump
  #   for more than 20s (to prevent pressure bleeding)
  if apply_brake > apply_brake_last or (ts - last_pump_ts > 20. and apply_brake > 0):
    last_pump_ts = ts

  # once the pump is on, run it for at least 0.2s
  if ts - last_pump_ts < 0.2 and apply_brake > 0:
    pump_on = True

  return pump_on, last_pump_ts

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.params = CarControllerParams(CP)
    self.ecar_can = EcarCAN(self.packer)
    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.stopping_counter = 0
    self.apply_angle_last = 0

    self.accel = 0.0
    self.speed = 0.0
    self.gas = 0.0
    self.brake = 0.0
    self.last_torque = 0.0

  def update_old(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    lat_active = CC.latActive

    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)

      can_sends.append(self.ecar_can.create_steering_control(self.apply_angle_last))

    if self.frame % 10 == 0:
      can_sends.append(self.ecar_can._brake_cmd_msg(0, 0x0))
      can_sends.append(self.ecar_can._park_cmd_msg(0))


    # print ("GearShifter", CS.out.gearShifter)
    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        # accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        can_sends.append(self.ecar_can.create_longitudinal_command(actuators.speed, CS.out.gearShifter))
    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        can_sends.append(self.ecar_can.create_longitudinal_command(CS.out.vEgo, CS.out.gearShifter))

    # TODO: HUD control
    # new_actuators = actuators.as_builder()
    new_actuators = structs.CarControl.Actuators()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.speed = CS.out.vEgo

    self.frame += 1
    return new_actuators, can_sends


  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    pcm_cancel_cmd = CC.cruiseControl.cancel

    if CC.longActive:
      accel = actuators.accel
      gas, brake = compute_gas_brake(actuators.accel, CS.out.vEgo)
    else:
      accel = 0.0
      gas, brake = 0.0, 0.0

    # *** rate limit steer ***
    limited_torque = rate_limit(actuators.torque, self.last_torque, -self.params.STEER_DELTA_DOWN * DT_CTRL,
                                self.params.STEER_DELTA_UP * DT_CTRL)
    self.last_torque = limited_torque

    # *** apply brake hysteresis ***
    pre_limit_brake, self.braking, self.brake_steady = actuator_hysteresis(brake, self.braking, self.brake_steady,
                                                                           CS.out.vEgo, self.CP.carFingerprint)

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(pre_limit_brake, self.brake_last, -2., DT_CTRL)
    # steer torque is converted back to CAN reference (positive when steering right)
    apply_torque = int(np.interp(-limited_torque * self.params.STEER_MAX,
                                 self.params.STEER_LOOKUP_BP, self.params.STEER_LOOKUP_V))

    print (f"{self.params.STEER_MAX=} {self.params.STEER_LOOKUP_BP=} {self.params.STEER_LOOKUP_V=}")

    # wind brake from air resistance decel at high speed
    wind_brake = np.interp(CS.out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15])
    # all of this is only relevant for HONDA ECAR
    max_accel = np.interp(CS.out.vEgo, self.params.ECAR_MAX_ACCEL_BP, self.params.ECAR_MAX_ACCEL_V)
    # TODO this 1.44 is just to maintain previous behavior
    pcm_speed_BP = [-wind_brake,
                    -wind_brake * (3 / 4),
                    0.0,
                    0.5]
    if not CC.longActive:
      pcm_speed = 0.0
      pcm_accel = int(0.0)
    else:
      pcm_speed_V = [0.0,
                     np.clip(CS.out.vEgo - 2.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 2.0, 0.0, 100.0),
                     np.clip(CS.out.vEgo + 5.0, 0.0, 100.0)]
      pcm_speed = float(np.interp(gas - brake, pcm_speed_BP, pcm_speed_V))
      pcm_accel = int(np.clip((accel / 1.44) / max_accel, 0.0, 1.0) * self.params.ECAR_GAS_MAX)
    self.speed = pcm_speed
    self.gas = pcm_accel / self.params.ECAR_GAS_MAX

      # Send gas and brake commands.
    if self.frame % 2 == 0:
      ts = self.frame * DT_CTRL

      apply_brake = np.clip(self.brake_last - wind_brake, 0.0, 1.0)
      apply_brake = int(np.clip(apply_brake * self.params.ECAR_BRAKE_MAX, 0, self.params.ECAR_BRAKE_MAX - 1))
      pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)
      self.apply_brake_last = apply_brake
      self.brake = apply_brake / self.params.ECAR_BRAKE_MAX

    #torque for steering control
    can_sends.append(self.ecar_can.create_steering_control(apply_torque))
    can_sends.append(self.ecar_can.create_longitudinal_command(self.speed, CS.out.gearShifter))
    can_sends.append(self.ecar_can._brake_cmd_msg(self.brake, 0x0))
    can_sends.append(self.ecar_can._park_cmd_msg(0))
    # TODO: HUD control
    # new_actuators = actuators.as_builder()
    new_actuators = structs.CarControl.Actuators()
    new_actuators.speed = self.speed
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas
    new_actuators.brake = self.brake
    new_actuators.torque = self.last_torque
    new_actuators.torqueOutputCan = apply_torque

    # print (f"{new_actuators=}")
    self.frame += 1
    return new_actuators, can_sends
