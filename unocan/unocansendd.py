from cereal import messaging
from openpilot.common.realtime import Ratekeeper
from unocan import CanHandle

RATE=50

class CanSend:
  def __init__(self) -> None:
    self.rk = Ratekeeper(RATE, print_delay_threshold=None)
    self.ch = CanHandle()
    self.sm = messaging.SubMaster(['sendcan'])

  def update(self):
    self.sm.update()
    if self.sm.valid['sendcan']:
      self.ch.can_send_many(self.sm["sendcan"], timeout=25)
      self.rk.keep_time()

  def cansend_thread(self):
    while True:
      self.update()

def main():
  cansendd = CanSend()
  cansendd.cansend_thread()

if __name__ == "__main__":
  main()