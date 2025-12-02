from unobox.sensor_bridge import SensorBridge
import rclpy

class UnoboxBridged:
  def __init__(self):
    self.sensor_bridge = SensorBridge()

  def run(self):
    rclpy.spin(self.sensor_bridge)

if __name__ == '__main__':
  rclpy.init()
  unobox_bridged = UnoboxBridged()
  unobox_bridged.run()