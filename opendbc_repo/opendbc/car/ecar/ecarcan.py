from opendbc.car.ecar.values import CarControllerParams, GEAR_INVERSE_MAP
from opendbc.car.ecar.values import CANBUS, CarControllerParams
from opendbc.car.common.conversions import Conversions as CV

class EcarCAN:
  def __init__(self, packer):
    self.packer = packer

  def create_steering_control(self, angle, active=1):
    values = {
      "ADCU_Str_Active": active,
      "ADCU_Str_CtrlMode": 0, #Angle mode is 0, (Reserved) Curvature mode is 1
      "ADCU_Str_TgtAngle": -angle, # left positive, right negative
    }
    print (f"ECAR Real Steer: {-angle} Deg!")
    return self.packer.make_can_msg("ADCU_SteerCmd", CANBUS.vehicle, values)

  def create_vehicle_dynamic_state(self, speed, run_direction):
    values = {
      "CDCU_Veh_LongtdnalSpd": speed, #kmph
      "CDCU_Veh_RunDir": run_direction,
    }

    return self.packer.make_can_msg("CDCU_VehDyncState", CANBUS.vehicle, values)

  def create_cdcu_steer_status(self, steer, steer_rate, steer_torque, work_mode):
    values = {
      "CDCU_EPS_StrWhlAngle": steer,
      "CDCU_EPS_WhlSpd": steer_rate,
      "CDCU_EPS_StrTrq": steer_torque,
      "CDCU_EPS_WorkMode": work_mode,
      "CDCU_EPS_ErrLevel": 0x0,
    }
    # print ("steer", values)
    return self.packer.make_can_msg("CDCU_SteerStatus", CANBUS.vehicle, values)

  def create_cdcu_brake_status(self, brake, work_mode):
    values = {
      "CDCU_EHB_BrkPedpos": brake,
      "CDCU_EHB_WorkMode": work_mode,
    }

    return self.packer.make_can_msg("CDCU_BrakeStatus", CANBUS.vehicle, values)

  def create_cdcu_drive_status(self, work_mode, gear_act, run_direction):
    values = {
      "CDCU_MCU_WorkMode": work_mode,
      "CDCU_MCU_GearAct": gear_act,
      "CDCU_MCU_RunDir": run_direction,
    }
    return self.packer.make_can_msg("CDCU_DriveStatus", CANBUS.vehicle, values)

  def create_longitudinal_command(self, v_ego, gear, active=1):
    set_speed = max(v_ego * CV.MS_TO_KPH, 0)
    values = {
      "ADCU_Drv_Active":active,
      "ADCU_Drv_CtrlMode": 1, #1: Speed control, 0: Throttle control
      "ADCU_Drv_TgtGear": GEAR_INVERSE_MAP[gear], #0: Neutral, 1: Drive, 2: Reverse,
      "ADCU_Drv_TgtVehSpd0": set_speed,
      "ADCU_Drv_VehSpdLimit": CarControllerParams.SPEED_MAX,
    }

    print (f"ECAR Real Speed: {set_speed} KM/H!")
    return self.packer.make_can_msg("ADCU_DriveCmd", CANBUS.vehicle, values)

  def _brake_cmd_msg(self, brake: float, pos_mode: int = 0, active=1):
    set_brake = brake*100
    values = {
      "ADCU_Brk_Active": active,
      "ADCU_Brk_CtrlMode": 0 if pos_mode else 1, #0 Brake Pedal Pos Mode, 1 Brake Pressure Mode
      "ADCU_Brk_TgtPedpos": set_brake, # [0,1] -> [0, 100]
      "ADCU_Brk_TgtPress": 0,
    }
    print (f"ECAR Real Brake: {set_brake} %!")
    return self.packer.make_can_msg("ADCU_BrakeCmd", CANBUS.vehicle, values)

  def _park_cmd_msg(self, enable: int, active=1):
    values = {
      "ADCU_Prk_Active": active,
      "ADCU_Prk_Enable": 1 if enable else 0,
    }
    return self.packer.make_can_msg("ADCU_ParkCmd", CANBUS.vehicle, values)

  def _power_info_msg(self, power_down: int = 0x0, charger_gun: int = 0x0):
    values = {
      "ADCU_AD12VMCPwrup_Cmd": charger_gun,
      "ADCU_ChasPwrdown_Req": power_down,
    }
    return self.packer.make_can_msg("ADCU_PowerInfo", CANBUS.vehicle, values)

class RollCount():
  roll_count = 0
  def __init__(self) -> None:
    self.roll_count = 0

  def update(self):
    self.roll_count = (self.roll_count + 1) % 16

  def get_value(self):
    return self.roll_count

def bitwise_sum(d):
  return sum(d[0:7])

def ecar_checksum(address: int, sig, d: bytearray) -> int:
  sum06= bitwise_sum(d)
  return (sum06 ^ 0xFF) & 0xFF

# def test_ecar_checksum(d: bytearray):
#   print ("raw integer", i, "bytearry", d, "byte0:byte6", d[0:5], "checksum", ecar_checksum(d))

# if __name__ == "__main__":
#   roll_count = RollCount()
#   for i in range(1, 2**9):
#     print (roll_count.get_value())
#     test_ecar_checksum(i.to_bytes(8, 'little'))
#     roll_count.update()