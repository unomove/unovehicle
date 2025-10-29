import traceback
import cereal.messaging as messaging

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car.ecar.values import EcarSafetyFlags, RunMode, GEAR, RunDirection
from opendbc.car.ecar.ecarcan import EcarCAN
from openpilot.common.params import Params
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from openpilot.tools.sim.lib.common import SimulatorState
from opendbc.car.common.conversions import Conversions as CV


class SimulatedCar:
  """Simulates a honda ecar (panda state + can messages) to OpenPilot"""
  packer = CANPacker("ecar_e70")

  def __init__(self):
    self.pm = messaging.PubMaster(['can', 'pandaStates'])
    self.sm = messaging.SubMaster(['carControl', 'controlsState', 'carParams', 'selfdriveState'])
    self.cp = self.get_car_can_parser()
    self.idx = 0
    self.params = Params()
    self.obd_multiplexing = False
    self.ecar_can = EcarCAN(self.packer)

  @staticmethod
  def get_car_can_parser():
    dbc_f = 'ecar_e70'
    checks = []
    return CANParser(dbc_f, checks, 0)

  def send_can_messages(self, simulator_state: SimulatorState):
    if not simulator_state.valid:
      return

    msg = []
    # *** powertrain bus ***

    # vechile state simulation:
    speed = simulator_state.speed * CV.MS_TO_KPH # convert m/s to kph
    steering_angle_deg = simulator_state.steering_angle * CV.RAD_TO_DEG
    brake = simulator_state.user_brake
    msg.append(self.ecar_can.create_cdcu_brake_status(brake, RunMode.AUTOMATIC))
    msg.append(self.ecar_can.create_cdcu_steer_status(steering_angle_deg,simulator_state.steering_angle , simulator_state.user_torque, RunMode.AUTOMATIC))
    msg.append(self.ecar_can.create_cdcu_drive_status(RunMode.AUTOMATIC, GEAR.D, RunDirection.FORWARD))
    msg.append(self.ecar_can.create_vehicle_dynamic_state(speed, RunDirection.FORWARD))

    self.pm.send('can', can_list_to_can_capnp(msg))

  def send_panda_state(self, simulator_state):
    self.sm.update(0)

    if self.params.get_bool("ObdMultiplexingEnabled") != self.obd_multiplexing:
      self.obd_multiplexing = not self.obd_multiplexing
      self.params.put_bool("ObdMultiplexingChanged", True)

    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    dat.pandaStates[0] = {
      'ignitionLine': simulator_state.ignition,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': 'ecar',
      'alternativeExperience': self.sm["carParams"].alternativeExperience,
      'safetyParam': EcarSafetyFlags.LONG_CONTROL.value,
    }
    self.pm.send('pandaStates', dat)

  def update(self, simulator_state: SimulatorState):
    try:
      self.send_can_messages(simulator_state)

      if self.idx % 50 == 0: # only send panda states at 2hz
        self.send_panda_state(simulator_state)

      self.idx += 1
    except Exception:
      traceback.print_exc()
      raise
