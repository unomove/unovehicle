from opendbc.car.ecar.values import DBC, GEAR, GEAR_MAP
from opendbc.can import CANDefine, CANPacker
from opendbc.car import Bus
from opendbc.car.ecar.ecarcan import EcarCAN
from opendbc.car.interfaces import CarStateBase
from opendbc.car.ecar.values import RunMode, GEAR, RunDirection

print (DBC)
# print (DBC[CP.carFingerprint][Bus.party])

def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.ecar.interface import CarInterface
  return CarInterface.get_non_essential_params("ECAR_E70")

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.main])
    self.shifter_values = self.can_define.dv["CDCU_VehDyncState"]["CDCU_Veh_RunDir"] #

# cs = CarState()

can_define = CANDefine(DBC["ECAR_E70"][Bus.main])
vl = can_define.dv["CDCU_VehState"]["CDCU_Veh_RunMode"]
print (vl)
print (vl.get(int(0)))

print (can_define.dv["CDCU_DriveStatus"]["CDCU_MCU_GearAct"])
print (can_define.dv["CDCU_DriveStatus"]["CDCU_MCU_GearAct"].get(int(0)))
print (GEAR_MAP[can_define.dv["CDCU_DriveStatus"]["CDCU_MCU_GearAct"].get(int(0))])

# print (get_safety_CP())
packer = CANPacker("ecar_e70")
ecar_can = EcarCAN(packer)
msg = []
idx = 0
brake, steering_angle, speed = 0, 0, 0
print (GEAR.N)
print (RunDirection.STOP)
msg.append(ecar_can.create_cdcu_brake_status(brake, RunMode.AUTOMATIC))
msg.append(ecar_can.create_cdcu_drive_status(RunMode.AUTOMATIC, GEAR.D, RunDirection.FORWARD))
msg.append(ecar_can.create_vehicle_dynamic_state(speed, RunDirection.FORWARD))

d = ecar_can.create_vehicle_dynamic_state(speed, RunDirection.FORWARD)
print (d)