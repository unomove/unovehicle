import time
from contextlib import AbstractContextManager

from unocan import CanHandle
from opendbc.car.car_helpers import get_car
from opendbc.car.can_definitions import CanData
from opendbc.car.structs import CarParams, CarControl, CarState
from tqdm import tqdm
import time

class UnocanRunner(AbstractContextManager):
  def __enter__(self):
    self.ch = CanHandle()
    self.CI = get_car(self._can_recv, self.ch.can_send_many, self.ch.set_obd, True, False)
    assert self.CI.CP.carFingerprint.lower() != "mock", "Unable to identify car. Check connections and ensure car is supported."
    self.CI.init(self.CI.CP, self._can_recv, self.ch.can_send_many)

    print ("Getting Ready for Autocontrol")
    can_sends=[
      self.CI.CC.ecar_can.create_steering_control(0,active=0),
      self.CI.CC.ecar_can.create_longitudinal_command(0, "neutral", active=0),
      self.CI.CC.ecar_can._brake_cmd_msg(0, 0x0,active=0),
      self.CI.CC.ecar_can._park_cmd_msg(0,active=0),
      self.CI.CC.ecar_can._bocy_cmd_msg(active=0),
      self.CI.CC.ecar_can._power_info_msg(),
    ]

    real_sends = [CanData(addr, dat, bus) for addr, dat, bus in can_sends]

    for i in tqdm(range(10)):
      self.ch.can_send_many(real_sends)
      time.sleep(0.05)
    return self

  def __exit__(self, exc_type, exc_value, traceback):
    self.ch.reset()  # avoid siren
    return super().__exit__(exc_type, exc_value, traceback)

  @property
  def handle(self) -> CanHandle:
    return self.ch

  def _can_recv(self, wait_for_one: bool = False) -> list[list[CanData]]:
    recv = self.ch.can_recv()
    while len(recv) == 0 and wait_for_one:
      recv = self.ch.can_recv()
    return [[CanData(addr, dat, bus) for addr, dat, bus in recv], ]

  def read(self, strict: bool = True):
    cs = self.CI.update([int(time.monotonic()*1e9), self._can_recv()[0]])
    if strict:
      assert cs.canValid, "CAN went invalid, check connections"
    return cs

  def write(self, cc: CarControl) -> None:
    if cc.enabled and not self.ch.health()['controls_allowed']:
      # prevent the car from faulting. print a warning?
      cc = CarControl(enabled=False)
    _, can_sends = self.CI.apply(cc)
    real_sends = [CanData(addr, dat, bus) for addr, dat, bus in can_sends]
    # print ("can sends", can_sends)
    self.ch.can_send_many(real_sends, timeout=25)

if __name__ == "__main__":
  with UnocanRunner() as uc:
    print(uc.read())
