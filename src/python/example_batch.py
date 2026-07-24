import numpy
import pyglim
from tqdm import tqdm
from pyridescence import *
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


class GlimExecutor:
  def __init__(self):
    self.batch_size = 1
    self.time_keeper = pyglim.BatchedTimeKeeper(batch_size=self.batch_size)
    self.preprocessor = pyglim.BatchedCloudPreprocessor(batch_size=self.batch_size)
    self.odometry = pyglim.BatchedOdometryEstimation(batch_size=self.batch_size, so_name='libodometry_estimation_cpu.so')

    with pyglim.ScopedCallbackContext(0):
      self.viewer = pyglim.ExtensionModule.load_module('libstandard_viewer.so')
    self.sub_viewers = []
    for i in range(self.batch_size):
      with pyglim.ScopedCallbackContext(i):
        sub_viewer = pyglim.ExtensionModule.load_module('libstandard_sub_viewer.so')
        self.sub_viewers.append(sub_viewer)

  def batch_imu_callback(self, batched_imu):
    imu = pyglim.BatchedIMUMeasurements(self.batch_size)
    imu.insert(batched_imu)

    self.time_keeper.validate_imu_stamp(imu)
    self.odometry.insert_imu(imu)

    print('--- IMU ---')
    print(batched_imu[0, :, 0])

  def batch_points_callback(self, stamps, points, times):
    batched_points = pyglim.BatchedRawPoints(self.batch_size)
    batched_points.set_stamps(stamps)
    batched_points.set_points(points)
    batched_points.set_times(times)

    self.time_keeper.process(batched_points)
    preprocessed = self.preprocessor.process(batched_points)
    latest_frame, marginalized_frames = self.odometry.insert_frame(preprocessed)

    print('--- Points ---')
    print(stamps)
    print(times)


def main():
  typestore = get_typestore(Stores.LATEST)

  config_path = '/home/koide/datasets/glim/config'
  bag_path = '/home/koide/datasets/glim/os1_128_01_downsampled'

  pyglim.GlobalConfig.instance(config_path)
  glim = GlimExecutor()

  all_imu_data = []
  all_points_data = []

  with Reader(bag_path) as reader:
    for connection, timestamp, rawdata in tqdm(reader.messages(), total=reader.message_count):
      if connection.topic == '/os_cloud_node/imu':
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        all_imu_data.append([stamp, *acc, *gyro])

      if connection.topic == '/os_cloud_node/points':
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        points_data = numpy.ascontiguousarray(msg.data.reshape(-1, msg.point_step)[:, :12])
        points = numpy.frombuffer(points_data, dtype=numpy.float32).reshape(-1, 3)

        times_data = numpy.ascontiguousarray(msg.data.reshape(-1, msg.point_step)[:, 12:16])
        times = numpy.frombuffer(times_data, dtype=numpy.float32).reshape(-1, 1)
        all_points_data.append((stamp, points, times))

  all_imu_data = numpy.float64(all_imu_data)

  points_shift = 5
  for i in range(1, len(all_points_data) - points_shift * glim.batch_size):
    batch = []
    for j in range(glim.batch_size):
      batch.append(all_points_data[i + j * points_shift])

    batched_stamps = [p[0] for p in batch]
    batched_points = [p[1] for p in batch]
    batched_times = [p[2] for p in batch]

    max_num_points = max(p.shape[0] for p in batched_points)
    batched_stamps = numpy.float64(batched_stamps)
    padded_points = numpy.full((glim.batch_size, max_num_points, 3), numpy.nan, dtype=numpy.float32)
    padded_times = numpy.full((glim.batch_size, max_num_points, 1), numpy.nan, dtype=numpy.float32)
    for j in range(glim.batch_size):
      num_points = batched_points[j].shape[0]
      padded_points[j, :num_points, :] = batched_points[j]
      padded_times[j, :num_points, :] = batched_times[j]

    prev_stamps = []
    for j in range(glim.batch_size):
      prev_stamps.append(all_points_data[i + j * points_shift - 1][0])
    prev_stamps = numpy.float64(prev_stamps)

    batch = []
    for j in range(glim.batch_size):
      imu_mask = (all_imu_data[:, 0] >= prev_stamps[j]) & (all_imu_data[:, 0] < batched_stamps[j])
      batch.append(all_imu_data[imu_mask])

    max_num_imu = max(b.shape[0] for b in batch)
    padded_imu = numpy.full((glim.batch_size, max_num_imu, 7), numpy.nan, dtype=numpy.float64)
    for j in range(glim.batch_size):
      num_imu = batch[j].shape[0]
      padded_imu[j, :num_imu, :] = batch[j]

    glim.batch_imu_callback(padded_imu)
    glim.batch_points_callback(batched_stamps, padded_points, padded_times)

  

if __name__ == "__main__":
  main()