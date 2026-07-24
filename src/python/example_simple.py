import numpy
import pyglim
from pyridescence import *
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


class GlimExecutor:
  def __init__(self):
    self.time_keeper = pyglim.TimeKeeper()
    self.preprocessor = pyglim.CloudPreprocessor()

    with pyglim.ScopedCallbackContext(0):
      self.odometry = pyglim.OdometryEstimationGPU()
      self.sub_mapping = pyglim.SubMappingPassthrough()
      self.global_mapping = pyglim.GlobalMapping()
      self.viewer = pyglim.ExtensionModule.load_module('libstandard_viewer.so')

  def imu_callback(self, stamp, acc, gyro):
    self.time_keeper.validate_imu_stamp(stamp)

    with pyglim.ScopedCallbackContext(0):
      self.odometry.insert_imu(stamp, acc, gyro)
      self.sub_mapping.insert_imu(stamp, acc, gyro)
      self.global_mapping.insert_imu(stamp, acc, gyro)

  def points_callback(self, stamp, points, times):
    with pyglim.ScopedCallbackContext(0):
      raw_points = pyglim.RawPoints(stamp, points, times.flatten())
      self.time_keeper.process(raw_points)
      preprocessed = self.preprocessor.preprocess(raw_points)
      latest_frame, marginalized_frames = self.odometry.insert_frame(preprocessed)

      if latest_frame is None:
        return

      self.sub_mapping.insert_frame(latest_frame)

      for submap in self.sub_mapping.get_submaps():
        self.global_mapping.insert_submap(submap)



def main():
  typestore = get_typestore(Stores.LATEST)

  config_path = '/home/koide/datasets/glim/config'
  bag_path = '/home/koide/datasets/glim/os1_128_01_downsampled'

  pyglim.GlobalConfig.instance(config_path)
  glim = GlimExecutor()

  with Reader(bag_path) as reader:
    for connection, timestamp, rawdata in reader.messages():
      if connection.topic == '/os_cloud_node/imu':
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        acc = numpy.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = numpy.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        glim.imu_callback(stamp, acc, gyro)

      if connection.topic == '/os_cloud_node/points':
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        points_data = numpy.ascontiguousarray(msg.data.reshape(-1, msg.point_step)[:, :12])
        points = numpy.frombuffer(points_data, dtype=numpy.float32).reshape(-1, 3)

        times_data = numpy.ascontiguousarray(msg.data.reshape(-1, msg.point_step)[:, 12:16])
        times = numpy.frombuffer(times_data, dtype=numpy.float32).reshape(-1, 1)

        glim.points_callback(stamp, points, times)
  

if __name__ == "__main__":
  main()