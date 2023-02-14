
import argparse
from scipy.spatial.transform import Rotation, Slerp
import time

import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting

import rospy
import rosbag
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import Imu, PointField
from std_msgs.msg import Header

from dataset_loaders import *
from utils import *

def heatmap_to_pointcloud(radar_hm, cfar_params):
    """

    :param radar_hm: elevation x azimuth x range x 2(intensity and range rate)
    :param cfar_params:
    :return: indices of detected points [ei, ai, ri]
    """
    hmshp = radar_hm.shape
    # normalize the heatmap in intensity
    maxval = radar_hm[:, :, :, 0].max()
    minval = radar_hm[:, :, :, 0].min()
    # print('intensity range is [{}, {}]'.format(minval, maxval))
    radar_hm[:, :, :, 0] = (radar_hm[:, :, :, 0] - minval) / (maxval - minval)
    indicators = np.zeros(hmshp[:3])
    half_range_guard = cfar_params['range_guard'] // 2
    half_azimuth_guard = cfar_params['azimuth_guard'] // 2
    half_elevation_guard = cfar_params['elevation_guard'] // 2
    rngstart = half_range_guard + cfar_params['range_train'] // 2
    rngend = hmshp[2] - rngstart
    azistart = half_azimuth_guard + cfar_params['azimuth_train'] // 2
    aziend = hmshp[1] - azistart
    elvstart = half_elevation_guard + cfar_params['elevation_train'] // 2
    elvend = hmshp[0] - elvstart
    N = (elvstart * 2 + 1) * (azistart * 2 + 1) * (rngstart * 2 + 1) - (cfar_params['elevation_guard'] + 1) * \
        (cfar_params['azimuth_guard'] + 1) * (cfar_params['range_guard'] + 1)

    # alpha = N * (math.pow(cfar_params['pfa'], -1/N) - 1)
    # print('alpha is {} for N {}'.format(alpha, N))
    start = time.time()
    pt_ids = []
    salient_intensities = []

    for i in range(elvstart, elvend):
        for j in range(azistart, aziend):
            for k in range(rngstart, rngend):
                if radar_hm[i,j,k,0] < cfar_params['intensity_threshold']:
                    continue
                # get values of the training cells
                maxval = 0
                maxids = []
                vals = np.zeros(N)
                valid = 0
                for ii in range(i - elvstart, i + elvstart + 1):
                    for jj in range(j - azistart, j + azistart + 1):
                        for kk in range(k - rngstart, k + rngstart + 1):
                            if radar_hm[ii, jj, kk, 0] > maxval:
                                maxval = radar_hm[ii, jj, kk, 0]
                                maxids = [ii, jj, kk]
                            if i - half_elevation_guard <= ii <= i + half_elevation_guard and \
                                    j - half_azimuth_guard <= jj <= j + half_azimuth_guard and \
                                    k - half_range_guard <= kk <= k + half_range_guard:
                                continue
                            else:
                                vals[valid] = radar_hm[ii, jj, kk, 0]
                                valid += 1
                assert(valid == N)
                med = np.median(vals)
                thresh = med * cfar_params['custom_threshold_factor']
                if radar_hm[i, j, k, 0] > thresh:  # and i == maxids[0] and j == maxids[1] and k == maxids[2]:
                    pt_ids.append((i, j, k))
                    indicators[i, j, k] = radar_hm[i, j, k, 0]
                    salient_intensities.append(radar_hm[i, j, k, 0])
    end1 = time.time()
    t1 = end1 - start

    # nonmaximum suppression in blocks of size 3x3x3
    status = np.ones(len(pt_ids))
    for c, ids in enumerate(pt_ids):
        i, j, k = ids
        val = indicators[i, j, k]
        if val == 0:
            continue
        for ii in range(i - 1, i + 2):
            for jj in range(j - 1, j + 2):
                for kk in range(k - 1, k + 2):
                    if indicators[ii, jj, kk] > val:
                        indicators[i, j, k] = 0
                        status[c] = 0
                        break
                if status[c] == 0:
                    break
            if status[c] == 0:
                break
    nms_ptids = [pt_ids[i] for i in range(len(pt_ids)) if status[i] == 1]
    print('Found {} peaks, NMS {} points, min salient point intensity {} with threshold factor {}.'.format(
        len(pt_ids), len(nms_ptids), np.min(salient_intensities), cfar_params['custom_threshold_factor']))
    end2 = time.time()
    t2= end2 - end1
    print('Elapsed time for peak detection {} s, NMS {} s'.format(t1, t2))
    return nms_ptids


def select_points(radar_pc_precalc, pt_ids, heatmapshape):
    """

    :param radar_pc_precalc: M x 5
    The points for close range has been discarded by min_range_idx.
    :param pt_ids: each row [elevation idx, azimuth idx, range idx]
    The range_idx has removed the min_range_idx.
    :param heatmapshape: [#elevation bins, #azimuth bins, #range bins - min_range_idx]
    :return: N x 5
    """
    ids = []
    steps = [heatmapshape[1] * heatmapshape[2], heatmapshape[2]]
    assert(steps[0] * heatmapshape[0] == radar_pc_precalc.shape[0])
    for id in pt_ids:
        ei, ai, ri = id
        arrayid = ei * steps[0] + ai * steps[1] + ri
        ids.append(arrayid)

    radar_pc_local = radar_pc_precalc[ids, :]
    return radar_pc_local

def save_imu_data(bag, imudata, imutime, seq_id, topic):
    imu = Imu()
    imu.header.frame_id = 'imu'
    imu.header.seq = seq_id
    imu.header.stamp = rospy.Time.from_sec(imutime)
    imu.linear_acceleration.x = imudata['accel'][0]
    imu.linear_acceleration.y = imudata['accel'][1]
    imu.linear_acceleration.z = imudata['accel'][2]
    imu.angular_velocity.x = imudata['gyro'][0]
    imu.angular_velocity.y = imudata['gyro'][1]
    imu.angular_velocity.z = imudata['gyro'][2]
    bag.write(topic, imu, t=imu.header.stamp)

def save_pointcloud(bag, radar_pc_local, scan_time, seq_id, topic):
    """radar_pc_local: N x 5, N is the number of points
    refer to https://github.com/tomas789/kitti2bag/blob/master/kitti2bag/kitti2bag.py"""
    header = Header()
    header.frame_id = 'cascade_radar'
    header.seq = seq_id
    header.stamp = rospy.Time.from_sec(scan_time)

    # fill pcl msg
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('i', 12, PointField.FLOAT32, 1),
              PointField('r', 16, PointField.FLOAT32, 1)]
    pcl_msg = pcl2.create_cloud(header, fields, radar_pc_local.astype(np.float32))
    bag.write(topic, pcl_msg, t=pcl_msg.header.stamp)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='get location for sequence and calibration')
  parser.add_argument('-s', '--seq', type=str, help='directory of the sequence')
  parser.add_argument('-c', '--calib', type=str, help='directory of calib data')
  parser.add_argument('-t', '--threshold', type=float, default=0.2,
                      help='intensity threshold for plotting heatmap points. '
                           'Refer to High Resolution Point Clouds from mmWave Radar '
                           'https://arxiv.org/pdf/2206.09273.pdf on more info about heatmap.')
  parser.add_argument('-mr', '--min_range', type=int, default=10,
                      help='if plotting heatmaps, minimum range bin to start plotting')
  args = parser.parse_args()

  all_radar_params = get_cascade_params(args.calib)
  radar_params = all_radar_params['heatmap']
  gt_params = get_groundtruth_params()
  # parameters for detecting peaks
  cfar_params = {
      'range_guard': 4,
      'range_train': 6,
      'azimuth_guard': 4,
      'azimuth_train': 4,
      'elevation_guard': 4,
      'elevation_train': 4,
      'custom_threshold_factor': 6.0,  # scale up the median threshold by this factor.
      'intensity_threshold': 0.01,  # skip points of less intensity.
      'pfa': 0.01,  # false alarm probability, not used for now.
  }
  print('CFAR parameters: {}'.format(cfar_params))
  # make extrinsic transforms as 4x4 transformation matrix
  radar_params['T_bs'] = np.eye(4)
  radar_params['T_bs'][:3,3] = radar_params['translation']
  radar_params['T_bs'][:3,:3] = Rotation.from_quat(radar_params['rotation']).as_dcm()

  # get plot legend labels
  radar_label = radar_params['sensor_type'] + ' ' + radar_params['data_type']
  gt_label = gt_params['sensor_type'] + ' ' + gt_params['data_type']

  # get timestamps for each sensor type
  radar_timestamps = get_timestamps(args.seq, radar_params)
  gt_timestamps = get_timestamps(args.seq, gt_params)

  # get groundtruth poses
  gt_poses = get_groundtruth(args.seq)

  # interpolate groundtruth poses for each sensor measurement
  radar_gt, radar_indices = interpolate_poses(gt_poses,
                                              gt_timestamps,
                                              radar_timestamps)

  # interleave radar and lidar measurements
  plot_data = []
  radar_idx = 0
  while radar_idx < len(radar_indices):
      plot_data.append((radar_timestamps[radar_indices[radar_idx]],
                        radar_gt[radar_idx],
                        radar_indices[radar_idx],
                        'radar'))
      radar_idx += 1

  print('Total frames {}'.format(len(plot_data)))
  radar_pc_precalc = get_heatmap_points(radar_params, args.min_range)

  fig = plt.figure()
  ax = fig.gca(projection='3d')
  ax.set_xlim((-10,10))
  ax.set_ylim((-10,10))
  ax.set_zlim((-10,10))
  ax.set_title(radar_label)

  outputbag = os.path.join(args.seq, 'cascade', 'pointclouds.bag')
  bag = rosbag.Bag(outputbag, 'w')
  topic = '/cascade/pointcloud'

  for i in range(0, len(plot_data)):
      # load full radar heatmap from file
      radar_hm = get_heatmap(plot_data[i][2], args.seq, radar_params)

      # assign intensity and doppler values to precalculated heatmap points
      # excluding points below the minimum range bin
      hmshp = list(radar_hm.shape)
      hmshp[2] -= args.min_range
      radar_pc_precalc[:, 3:] = radar_hm[:, :, args.min_range:, :].reshape(-1, 2)

      pt_ids = heatmap_to_pointcloud(radar_hm[:, :, args.min_range:, :], cfar_params)
      radar_pc_local = select_points(radar_pc_precalc, pt_ids, hmshp)

      # downsample using voxel grid filter for faster plotting
      # radar_pc_local = downsample_pointcloud(radar_pc_precalc, 0.3)

      R_wb = np.array(plot_data[i][1])
      t_wb = np.array(R_wb[:3,3])
      R_wb[:3,3] = 0.0

      # transform to plotting frame
      T_ws = np.dot(R_wb, radar_params['T_bs'])
      radar_pc = transform_pcl(radar_pc_local, T_ws)
      ax.scatter(radar_pc[:, 0],
                 radar_pc[:, 1],
                 radar_pc[:, 2],
                 c=radar_pc[:, 3],
                 marker='.',
                 s=10,
                 label=radar_label)
      radar_time = plot_data[i][0]
      save_pointcloud(bag, radar_pc_local, radar_time, i, topic)
      plt.pause(0.2)
  print('Saved {} point clouds.'.format(len(plot_data)))
  imudata = get_imu(args.seq)
  imuparams = {'sensor_type': 'imu'}
  imutimes = get_timestamps(args.seq, imuparams)
  assert(len(imudata) == len(imutimes))
  imutopic = '/gx5/imu/data'
  for mi in range(len(imudata)):
      save_imu_data(bag, imudata[mi], imutimes[mi], mi, imutopic)
  print('Saved {} imu data.'.format(len(imudata)))
  bag.close()
