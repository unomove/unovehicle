import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car.ecar.ecarcan import EcarCAN
from openpilot.common.params import Params

class UnoboxBridge:
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