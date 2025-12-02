#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, Image
# TODO: adjust this import and type to match /bst_gps exactly
from sensor_msgs.msg import NavSatFix  # or your custom GPS msg

from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

# cereal
from cereal import log
import cereal.messaging as messaging
from msgq.visionipc import VisionIpcServer, VisionStreamType
from common.params import Params
from unobox.misc import W, H
import copy

from common.realtime import DT_DMON

class SensorBridge(Node):
  def __init__(self, dual_camera=False):
    super().__init__('sensor_bridge')
    self.vis = False
    self.bridge = CvBridge()
    self.obd_multiplexing = False
    self.params = Params()

    self.pm = messaging.PubMaster(["pandaStates", 'accelerometer', 'gyroscope', 'gpsLocationExternal', 'driverStateV2', 'driverMonitoringState', 'peripheralState', 'roadCameraState', 'wideRoadCameraState'])
    self.sm = messaging.SubMaster(['carControl', 'controlsState', 'carParams', 'selfdriveState'])

    self.bearing_deg = 0
    self.speed = 0
    self.velNED = [0, 0, 0]

    self.frame_road_id = 0
    self.frame_wide_id = 0
    self.vipc_server = VisionIpcServer("camerad")
    self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_ROAD, 5, W, H)
    if dual_camera:
      self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_WIDE_ROAD, 5, W, H)

    self.vipc_server.start_listener()

    # IMU
    self.imu_sub = self.create_subscription(
        Imu,
        '/imu/data_raw',
        self.imu_callback,
        10
    )

    # IMU
    self.imu_mag_sub = self.create_subscription(
        Imu,
        '/imu/mag',
        self.imu_mag_callback,
        10
    )

    # GPS
    self.gps_sub = self.create_subscription(
        NavSatFix,           # change if /bst_gps uses a different msg
        '/bst_gps',
        self.gps_callback,
        10
    )

    # Image
    self.image_sub = self.create_subscription(
        Image,
        '/camera4/image/raw',
        self.image_callback,
        10
    )

  def cam_send_yuv_road(self, yuv):
    self._send_yuv(yuv, self.frame_road_id, 'roadCameraState', VisionStreamType.VISION_STREAM_ROAD)
    self.frame_road_id += 1
    if self.frame_road_id % 10 == 0:
        self.send_panda_state()

  def cam_send_yuv_wide_road(self, yuv):
    self._send_yuv(yuv, self.frame_wide_id, 'wideRoadCameraState', VisionStreamType.VISION_STREAM_WIDE_ROAD)
    self.frame_wide_id += 1

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type):
    yuv = yuv.tobytes()
    eof = int(frame_id * 0.05 * 1e9)
    self.vipc_server.send(yuv_type, yuv, frame_id, eof, eof)

    dat = messaging.new_message(pub_type, valid=True)
    msg = {
      "frameId": frame_id,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
    }
    setattr(dat, pub_type, msg)
    self.pm.send(pub_type, dat)

  def imu_mag_callback(self, msg: Imu):
      # Extract quaternion data
      orientation_q = msg.orientation

      # Check if orientation covariance is valid (optional but recommended)
      # If all zeros, orientation data might not be available/reliable
      if all(c == 0.0 for c in msg.orientation_covariance):
          self.get_logger().warn('IMU orientation data covariance is zero/unknown. Bearing might be unreliable.')

      # Convert quaternion to Euler angles (roll, pitch, yaw)
      # Angles are in radians, typically roll and pitch are referenced to gravity, yaw may drift
      try:
          quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
          roll, pitch, yaw = euler_from_quaternion(quat)

          # Convert yaw (heading) from radians to degrees
          # Yaw is usually the rotation around the Z-axis
          bearing_deg = math.degrees(yaw)

          # Normalize bearing to 0-360 degrees
          # The 'bearing' in navigation is usually a positive value from North (0 deg) clockwise
          # The exact normalization depends on your sensor's specific "world frame" convention
          # A common approach for navigation bearing is:
          if bearing_deg < 0:
              bearing_deg += 360
          self.bearing_deg = bearing_deg

          self.get_logger().info(f'Received IMU data: Bearing (Heading): {bearing_deg:.2f} degrees')

      except Exception as e:
          self.get_logger().error(f'Error processing IMU data: {e}')

  def imu_callback(self, msg: Imu):
      self.get_logger().info(
          f'IMU lin_acc: x={msg.linear_acceleration.x:.3f}, '
          f'y={msg.linear_acceleration.y:.3f}, '
          f'z={msg.linear_acceleration.z:.3f}'
      )

      dat = messaging.new_message('accelerometer', valid=True)
      dat.accelerometer.sensor = 4
      dat.accelerometer.type = 0x10
      dat.accelerometer.timestamp = dat.logMonoTime  # TODO: use the IMU timestamp
      dat.accelerometer.init('acceleration')
      dat.accelerometer.acceleration.v = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
      self.pm.send('accelerometer', dat)

      # copied these numbers from locationd
      dat = messaging.new_message('gyroscope', valid=True)
      dat.gyroscope.sensor = 5
      dat.gyroscope.type = 0x10
      dat.gyroscope.timestamp = dat.logMonoTime  # TODO: use the IMU timestamp
      dat.gyroscope.init('gyroUncalibrated')
      dat.gyroscope.gyroUncalibrated.v = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
      self.pm.send('gyroscope', dat)

  def gps_callback(self, msg: NavSatFix):
      self.get_logger().info(
          f'GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}'
      )

      # todo venNED and speed should be set from the vhichle state

      dat = messaging.new_message('gpsLocationExternal', valid=True)
      dat.gpsLocationExternal = {
        "unixTimestampMillis": int(time.time() * 1000),  # noqa: TID251
        "flags": 1,  # valid fix
        "horizontalAccuracy": 1.0,
        "verticalAccuracy": 1.0,
        "speedAccuracy": 0.1,
        "bearingAccuracyDeg": 0.1,
        "vNED": self.velNED,
        "bearingDeg": self.bearing_deg,
        "latitude": msg.latitude,
        "longitude": msg.longitude,
        "altitude": msg.altitude,
        "speed": self.speed,
        "source": log.GpsLocationData.SensorSource.ublox,
      }

      self.pm.send('gpsLocationExternal', dat)

  def image_callback(self, msg: Image):
      # Convert ROS Image message to OpenCV image
      try:
          # Handle NV12 encoding (YUV 4:2:0 format)
          if msg.encoding == 'nv12':
              # NV12 format: Y plane (full res) + interleaved UV plane (half res)
              height = msg.height
              width = msg.width

              # Convert raw data to numpy array
              img_data = np.frombuffer(msg.data, dtype=np.uint8)

              # NV12 format: Y plane is width*height, then interleaved UV is width*height/2
              # OpenCV expects NV12 as a single array of shape (height * 3 // 2, width)
              # The first height rows are Y, the remaining height/2 rows are interleaved UV
              yuv_image = img_data.reshape((height * 3 // 2, width))
              vis_yuv = copy.deepcopy(yuv_image)
              self.cam_send_yuv_road(yuv_image)
              if self.vis:
                # Convert NV12 to BGR using OpenCV
                cv_image = cv2.cvtColor(vis_yuv, cv2.COLOR_YUV2BGR_NV12)
                cv2.imshow('camera0', cv_image)
                cv2.waitKey(1)
          else:
              # Use cv_bridge for other encodings
              cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


      except Exception as e:
          self.get_logger().error(f'Image conversion failed: {e}')

  def send_panda_state(self):
    self.sm.update(0)

    if self.params.get_bool("ObdMultiplexingEnabled") != self.obd_multiplexing:
      self.obd_multiplexing = not self.obd_multiplexing
      self.params.put_bool("ObdMultiplexingChanged", True)

    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    dat.pandaStates[0] = {
      'ignitionLine': True,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': 'ecar',
      'alternativeExperience': self.sm["carParams"].alternativeExperience,
      'safetyParam': 1,
    }
    self.pm.send('pandaStates', dat)

