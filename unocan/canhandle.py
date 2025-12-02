from .utils import logger
import can

from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from opendbc.car.common.conversions import Conversions as CV
import cereal.messaging as messaging
from opendbc.car.can_definitions import CanData

class CanHandle:
  # TODO: implement can_send_many and can_recv
  def __init__(self, channel: str = "can0", bus: int = 0, fd: bool = False):
    self.interface = "socketcan"
    self.channel = channel
    self.bus = bus
    self.fd = fd
    # self.can_logger = can.Logger(filename='can_log.asc', append=False)
    self.bus = can.Bus(interface=self.interface, channel=self.channel, bitrate=500000)
    self.pm = messaging.PubMaster(['can'])
    # notifier = can.Notifier(self.bus, [self.can_logger, can.Printer()])

  def can_send(self, address: int, data: bytes, bus: int = 0):
    self.can_send_many([(address, data, bus)])

  def can_send_many(self, messages: list[tuple[int, bytes, int]], timeout: int = 25):
    # logger.info(f"Sending {len(messages)} messages {messages}")
    logger.info(f"Sending {len(messages)} messages")
    for msg in messages:
      # print ("sending msg", msg)
      can_msg = can.Message(arbitration_id=msg.address, data=msg.dat, is_extended_id=False, is_rx=False, channel=self.channel)
      # print ("sent ", can_msg)
      self.bus.send(can_msg)

  def set_obd(self, obd):
    pass
    # self._handle.controlWrite(Panda.REQUEST_OUT, 0xdb, int(obd), 0, b'')

  def health(self):
    return {
      'controls_allowed': True,
    }

  def can_recv(self):
    # self.can_recv()
    # logger.info("Receiving messages")
    # debug use
    # return [(592, b'\x00\x00\x00\x00\x00\x00@\x00', 0)]
    msg = self.bus.recv(0.1)
    msg_list = []
    if msg is None:
      msg_list = [] # empty list
    else:
      msg_list.append((msg.arbitration_id, msg.data, 0))
    # print ("receive can messages from unocan board from Ecar E70", len(msg_list))
    # print (msg_list)
    self.pm.send("can", can_list_to_can_capnp(msg_list))
    # print (msg)
    return msg_list

  def reset(self):
    logger.info("Resetting CAN handle")