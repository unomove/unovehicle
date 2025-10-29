import traceback
import cereal.messaging as messaging

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from openpilot.common.params import Params
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from openpilot.tools.sim.lib.common import SimulatorState
from opendbc.car.common.conversions import Conversions as CV

class SimulatedCar:
  """Simulates a honda civic 2022 (panda state + can messages) to OpenPilot"""
  packer = CANPacker("tesla_model3_party")

  def __init__(self):
    self.pm = messaging.PubMaster(['can', 'pandaStates'])
    self.sm = messaging.SubMaster(['carControl', 'controlsState', 'carParams', 'selfdriveState'])
    self.cp = self.get_car_can_parser()
    self.idx = 0
    self.params = Params()
    self.obd_multiplexing = False

  @staticmethod
  def get_car_can_parser():
    dbc_f = 'tesla_model3_party'
    checks = []
    return CANParser(dbc_f, checks, 0)

  def send_can_messages(self, simulator_state: SimulatorState):
    if not simulator_state.valid:
      return

    msg = []

    # *** powertrain bus ***

    speed = simulator_state.speed * 3.6 # convert m/s to kph
    msg.append(self.packer.make_can_msg("DI_speed", 0, {"DI_vehicleSpeed": speed}))
    msg.append(self.packer.make_can_msg("DI_systemStatus", 0, {"DI_accelPedalPos": simulator_state.user_torque}))
    brakePressed = 2 if simulator_state.user_brake > 0 else 0
    msg.append(self.packer.make_can_msg("IBST_status", 0, {"IBST_driverBrakeApply": brakePressed}))
    values = {
      "EPAS3S_handsOnLevel": 1,
      "EPAS3S_internalSAS": simulator_state.steering_angle * CV.RAD_TO_DEG,
      "EPAS3S_eacStatus": 0,
      "EPAS3S_torsionBarTorque":simulator_state.user_torque,
    }
    msg.append(self.packer.make_can_msg("EPAS3S_sysStatus", 0, values))

    msg.append(self.packer.make_can_msg("SCCM_steeringAngleSensor", 0, {"SCCM_steeringAngleSpeed": simulator_state.steering_angle}))

    values = {
      "DI_cruiseState":  0,
      "DI_speedUnits": 1,
      "DI_digitalSpeed": speed,
    }
    msg.append(self.packer.make_can_msg("DI_state", 0, values))

    msg.append(self.packer.make_can_msg("DI_systemStatus", 0, {'DI_gear': 4}))

    values = {
      "DAS_blindSpotRearLeft" : simulator_state.left_blinker,
      "DAS_blindSpotRearRight" : simulator_state.right_blinker,
    }

    msg.append(self.packer.make_can_msg("DAS_status", 0, values))

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
      'safetyModel': 'hondaBosch',
      'alternativeExperience': self.sm["carParams"].alternativeExperience,
      # 'safetyParam': HondaSafetyFlags.RADARLESS.value | HondaSafetyFlags.BOSCH_LONG.value,
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
