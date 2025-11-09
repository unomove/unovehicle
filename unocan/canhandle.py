from .utils import logger

CANPACKET_HEAD_SIZE = 0x6
DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
LEN_TO_DLC = {length: dlc for (dlc, length) in enumerate(DLC_TO_LEN)}

def calculate_checksum(data):
  res = 0
  for b in data:
    res ^= b
  return res

def pack_can_buffer(arr, chunk=False, fd=False):
  snds = [bytearray(), ]
  for address, dat, bus in arr:
    extended = 1 if address >= 0x800 else 0
    data_len_code = LEN_TO_DLC[len(dat)]
    header = bytearray(CANPACKET_HEAD_SIZE)
    word_4b = (address << 3) | (extended << 2)
    header[0] = (data_len_code << 4) | (bus << 1) | int(fd)
    header[1] = word_4b & 0xFF
    header[2] = (word_4b >> 8) & 0xFF
    header[3] = (word_4b >> 16) & 0xFF
    header[4] = (word_4b >> 24) & 0xFF
    header[5] = calculate_checksum(header[:5] + dat)

    snds[-1].extend(header)
    snds[-1].extend(dat)
    if chunk and len(snds[-1]) > 256:
      snds.append(bytearray())

  return snds

def unpack_can_buffer(dat):
  ret = []

  while len(dat) >= CANPACKET_HEAD_SIZE:
    data_len = DLC_TO_LEN[(dat[0]>>4)]

    header = dat[:CANPACKET_HEAD_SIZE]

    bus = (header[0] >> 1) & 0x7
    address = (header[4] << 24 | header[3] << 16 | header[2] << 8 | header[1]) >> 3

    if (header[1] >> 1) & 0x1:
      # returned
      bus += 128
    if header[1] & 0x1:
      # rejected
      bus += 192

    # we need more from the next transfer
    if data_len > len(dat) - CANPACKET_HEAD_SIZE:
      break

    assert calculate_checksum(dat[:(CANPACKET_HEAD_SIZE+data_len)]) == 0, "CAN packet checksum incorrect"

    data = dat[CANPACKET_HEAD_SIZE:(CANPACKET_HEAD_SIZE+data_len)]
    dat = dat[(CANPACKET_HEAD_SIZE+data_len):]

    ret.append((address, data, bus))

  return (ret, dat)

class CanHandle:
  # TODO: implement can_send_many and can_recv
  def __init__(self, interface: str = "can0", bus: int = 0, fd: bool = False):
    self.interface = interface
    self.bus = bus
    self.fd = fd

  def can_send(self, address: int, data: bytes, bus: int = 0):
    self.can_send_many([(address, data, bus)])

  def can_send_many(self, messages: list[tuple[int, bytes, int]], timeout: int = 25):
    # logger.info(f"Sending {len(messages)} messages {messages}")
    logger.info(f"Sending {len(messages)} messages")

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
    return [(592, b'\x00\x00\x00\x00\x00\x00@\x00', 0)]

  def reset(self):
    logger.info("Resetting CAN handle")