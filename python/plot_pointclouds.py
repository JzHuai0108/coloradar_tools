from dataset_loaders import *
import math
import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation, Slerp
import argparse
from time import time


# interpolates poses for the given timestamps
# param[in] src_poses: list of poses in the form of a dict {'position': [x,y,z], 'orientation: [x,y,z,w]'}
# param[in] src_stamps: list of timestamps for the src_poses
# praam[in] tgt_stamps: list of timestamps for which poses need to be calculated
# return tgt_poses: list of interpolated poses as 4x4 transformation matrices
# return tgt_indices: list of indices in tgt_stamps for which poses were able to be interpolated
def interpolate_poses(src_poses, src_stamps, tgt_stamps):

  src_start_idx = 0
  tgt_start_idx = 0
  src_end_idx = len(src_stamps) - 1
  tgt_end_idx = len(tgt_stamps) - 1

  # ensure first source timestamp is immediately before first target timestamp
  while tgt_start_idx < tgt_end_idx and tgt_stamps[tgt_start_idx] < src_stamps[src_start_idx]:
    tgt_start_idx += 1

  # ensure last source timestamp is immediately after last target timestamp
  while tgt_end_idx > tgt_start_idx and tgt_stamps[tgt_end_idx] > src_stamps[src_end_idx]:
    tgt_end_idx -= 1

  # iterate through target timestamps, 
  # interpolating a pose for each as a 4x4 transformation matrix
  tgt_idx = tgt_start_idx
  src_idx = src_start_idx
  tgt_poses = []
  while tgt_idx <= tgt_end_idx and src_idx <= src_end_idx:
    # find source timestamps bracketing target timestamp
    while src_idx + 1 <= src_end_idx and src_stamps[src_idx + 1] < tgt_stamps[tgt_idx]:
      src_idx += 1

    # get interpolation coefficient
    c = ((tgt_stamps[tgt_idx] - src_stamps[src_idx]) 
          / (src_stamps[src_idx+1] - src_stamps[src_idx]))

    # interpolate position
    pose = np.eye(4)
    pose[:3,3] = ((1.0 - c) * src_poses[src_idx]['position'] 
                        + c * src_poses[src_idx+1]['position'])

    # interpolate orientation
    r_src = Rotation.from_quat([src_poses[src_idx]['orientation'],
                            src_poses[src_idx+1]['orientation']])
    slerp = Slerp([0,1],r_src)
    pose[:3,:3] = slerp([c])[0].as_dcm()

    tgt_poses.append(pose)

    # advance target index
    tgt_idx += 1

  tgt_indices = range(tgt_start_idx, tgt_end_idx + 1)
  return tgt_poses, tgt_indices

# downsamples a pointcloud for faster plotting using a voxel grid
# output pointcloud will have at most one point in a given voxel
# param[in] in_pcl: the pointcloud to be downsampled
# param[in] vox_size: the voxel size
# return out_pcl: the downsampled pointcloud
def downsample_pointcloud(in_pcl, vox_size):
  _, idx = np.unique((in_pcl[:,:3] / vox_size).round(),return_index=True,axis=0)
  out_pcl = in_pcl[idx,:]
  return out_pcl

# converts a bin location in a polar-formatted heatmap to a point in 
# cartesian space defined in meters
# param[in] r_bin: range bin index
# param[in] az_bin: azimuth bin index
# param[in] el_bin: elevation bin index
# param[in] params: heatmap parameters for the sensor
# return point: the point in cartesian coordinates
def polar_to_cartesian(r_bin, az_bin, el_bin, params):
  point = np.zeros(3)
  point[0] = (r_bin * params['range_bin_width'] 
              * math.cos(params['elevation_bins'][el_bin]) 
              * math.cos(params['azimuth_bins'][az_bin]))
  point[1] = (r_bin * params['range_bin_width']
              * math.cos(params['elevation_bins'][el_bin])
              * math.sin(params['azimuth_bins'][az_bin]))
  point[2] = (r_bin * params['range_bin_width']
              * math.sin(params['elevation_bins'][el_bin]))
  return point

# calculates point locations in the sensor frame for plotting heatmaps
# param[in] params: heatmap parameters for the sensor
# return pcl: the heatmap point locations
def get_heatmap_points(params):

  # transform range-azimuth-elevation heatmap to pointcloud
  pcl = np.zeros([params['num_elevation_bins'],
                  params['num_azimuth_bins'],
                  params['num_range_bins'] - args.min_range,
                  5])

  for range_idx in range(params['num_range_bins'] - args.min_range):
    for az_idx in range(params['num_azimuth_bins']):
      for el_idx in range(params['num_elevation_bins']):
        pcl[el_idx,az_idx,range_idx,:3] = polar_to_cartesian(range_idx + args.min_range, az_idx, el_idx, params)

  pcl = pcl.reshape(-1,5)
  return pcl

# performs a rigid transformation on a pointcloud
# param[in] pcl: the input pointcloud to be transformed
# param[in] T: the 4x4 rigid transformation matrix
# return out_points: the transformed pointcloud
def transform_pcl(pcl, T):
  in_points = pcl[:,:3]
  in_points = np.concatenate((in_points,np.ones((in_points.shape[0],1))), axis=1)
  out_points = np.dot(T,np.transpose(in_points))
  out_points = np.transpose(out_points[:3,:])
  if pcl.shape[1] > 3:
    out_points = np.concatenate((out_points,pcl[:,3:]), axis=1)
  return out_points

# animates sensor measurements for your enjoyment and edification
def animate_plot(i):
  global lidar_pc
  global radar_pc
  global radar_pc_precalc
  global plot_data
  global positions
  global ax

  # break up groundtruth transform into rotation and translation
  R_wb = np.array(plot_data[i][1])
  t_wb = np.array(R_wb[:3,3])
  R_wb[:3,3] = 0.0

  # determine if the lidar or radar plot is replaced
  # and load new plot in base sensor rig frame
  if plot_data[i][3] == 'lidar':

    # load lidar pointcloud from file
    lidar_pc_local = get_pointcloud(plot_data[i][2], args.seq, lidar_params)

    # downsample for faster plotting
    lidar_pc_local = downsample_pointcloud(lidar_pc_local, 0.3)

    # transform pointcloud to plotting frame
    T_ws = np.dot(R_wb, lidar_params['T_bs'])
    lidar_pc = transform_pcl(lidar_pc_local, T_ws)

  else:
    if args.plot_heatmap:

      # load full radar heatmap from file
      radar_hm = get_heatmap(plot_data[i][2], args.seq, radar_params)

      # assign intensity and doppler values to precalculated heatmap points
      # excluding points below the minimum range bin
      radar_pc_precalc[:,3:] = radar_hm[:,:,args.min_range:,:].reshape(-1,2)

      # downsample using voxel grid filter for faster plotting
      radar_pc_local = downsample_pointcloud(radar_pc_precalc, 0.3)

      # normalize intensity values 
      radar_pc_local[:,3] -= radar_pc_local[:,3].min()
      radar_pc_local[:,3] /= radar_pc_local[:,3].max()

      # remove points with intensity below the threshold value
      radar_pc_local = radar_pc_local[radar_pc_local[:,3] > args.threshold]

      # re-normalize intensity values after removing below-threshold points
      radar_pc_local[:,3] -= radar_pc_local[:,3].min()
      radar_pc_local[:,3] /= radar_pc_local[:,3].max()

    else:

      # load radar pointcloud from file
      radar_pc_local = get_pointcloud(plot_data[i][2], args.seq, radar_params)
    
    # transform to plotting frame
    T_ws = np.dot(R_wb, radar_params['T_bs'])
    radar_pc = transform_pcl(radar_pc_local, T_ws)

  # translate pointcloud that wasn't updated
  if i > 1:
    if plot_data[i][3] == 'lidar':
      radar_pc[:,:3] += (positions[-2] - positions[-1])
    else:
      lidar_pc[:,:3] += (positions[-2] - positions[-1])

  # add current position to plotted path
  positions = np.concatenate((positions, t_wb.reshape(1,3)),axis=0)
  positions_ctr = positions - t_wb

  # update plot data
  plots['positions'][0]._verts3d = (positions_ctr[:,0],
                                    positions_ctr[:,1],
                                    positions_ctr[:,2])

  plots['lidar']._offsets3d = (lidar_pc[:,0],
                               lidar_pc[:,1],
                               lidar_pc[:,2])

  plots['radar']._offsets3d = (radar_pc[:,0],
                            radar_pc[:,1],
                            radar_pc[:,2])
  plots['radar'].set_array(radar_pc[:,3])
  plots['radar'].set_cmap('plasma')
  

if __name__ == '__main__':

  parser = argparse.ArgumentParser(description='get location for sequence and calibration')
  parser.add_argument('-s', '--seq', type=str, help='directory of the sequence')
  parser.add_argument('-c', '--calib', type=str, help='directory of calib data')
  parser.add_argument('-t', '--threshold', type=float, default=0.2, help='intensity threshold for plotting heatmap points')
  parser.add_argument('-mr', '--min_range', type=int, default=10, help='if plotting heatmaps, minimum range bin to start plotting')
  parser.add_argument('-hm', '--plot_heatmap', type=str, default='false', help='true to plot radar heatmaps, false to plot pointclouds')
  parser.add_argument('-sc', '--single_chip', type=str, default='true', help='true to plot single chip data, false to plot cascade data')
  args = parser.parse_args()

  args.single_chip = args.single_chip.lower() == 'true'
  args.plot_heatmap = args.plot_heatmap.lower() == 'true'

  if not args.single_chip and not args.plot_heatmap:
    print('warn: pointclouds not available for cascade sensor, setting --plot_heatmap to True')
    args.plot_heatmap = True

  # get config parameters for each sensor type
  if args.single_chip:
    all_radar_params = get_single_chip_params(args.calib)
  else:
    all_radar_params = get_cascade_params(args.calib)

  if args.plot_heatmap:
    radar_params = all_radar_params['heatmap']
  else:
    radar_params = all_radar_params['pointcloud']

  gt_params = get_groundtruth_params()
  lidar_params = get_lidar_params(args.calib)

  # make extrinsic transforms as 4x4 transformation matrix
  radar_params['T_bs'] = np.eye(4)
  radar_params['T_bs'][:3,3] = radar_params['translation']
  radar_params['T_bs'][:3,:3] = Rotation.from_quat(radar_params['rotation']).as_dcm()

  lidar_params['T_bs'] = np.eye(4)
  lidar_params['T_bs'][:3,3] = lidar_params['translation']
  lidar_params['T_bs'][:3,:3] = Rotation.from_quat(lidar_params['rotation']).as_dcm()

  # get plot legend labels
  radar_label = radar_params['sensor_type'] + ' ' + radar_params['data_type']
  lidar_label = lidar_params['sensor_type'] + ' ' + lidar_params['data_type']
  gt_label = gt_params['sensor_type'] + ' ' + gt_params['data_type']

  # get timestamps for each sensor type
  radar_timestamps = get_timestamps(args.seq, radar_params)
  gt_timestamps = get_timestamps(args.seq, gt_params)
  lidar_timestamps = get_timestamps(args.seq, lidar_params)

  # get groundtruth poses
  gt_poses = get_groundtruth(args.seq)

  # interpolate groundtruth poses for each sensor measurement
  radar_gt, radar_indices = interpolate_poses(gt_poses, 
                                              gt_timestamps, 
                                              radar_timestamps)
  lidar_gt, lidar_indices = interpolate_poses(gt_poses, 
                                              gt_timestamps, 
                                              lidar_timestamps)

  # interleave radar and lidar measurements
  plot_data = []
  radar_idx = 0
  lidar_idx = 0
  while radar_idx < len(radar_indices) or lidar_idx < len(lidar_indices):
    if radar_idx < len(radar_indices) and radar_timestamps[radar_indices[radar_idx]] < lidar_timestamps[lidar_indices[lidar_idx]]:
      plot_data.append((radar_timestamps[radar_indices[radar_idx]],
                        radar_gt[radar_idx],
                        radar_indices[radar_idx],
                        'radar'))
      radar_idx += 1
    elif lidar_idx < len(lidar_indices):
      plot_data.append((lidar_timestamps[lidar_indices[lidar_idx]],
                        lidar_gt[lidar_idx],
                        lidar_indices[lidar_idx],
                        'lidar'))
      lidar_idx += 1

  # if using heatmap precalculate point locations because they're always the same
  # and there's a ton of them
  if args.plot_heatmap: 
    radar_pc_precalc = get_heatmap_points(radar_params)
  else:
    radar_pc_precalc = np.random.rand(10,4)

  radar_pc = np.random.rand(10,4)
  lidar_pc = np.random.rand(10,4)
  positions = np.zeros((1,3))

  # initialize plot
  fig = plt.figure()
  ax = fig.gca(projection='3d')
  ax.set_xlim((-10,10))
  ax.set_ylim((-10,10))
  ax.set_zlim((-10,10))
  ax.set_title('lidar and ' + radar_label)
  plots = {}

  plots['radar'] = ax.scatter(radar_pc[:,0],
                              radar_pc[:,1],
                              radar_pc[:,2],
                              c = [],
                              marker='.',
                              s=10,
                              label=radar_label)
  
  plots['lidar'] = ax.scatter(lidar_pc[:,0],
                              lidar_pc[:,1],
                              lidar_pc[:,2],
                              c='r',
                              marker='.',
                              s=0.5,
                              label=lidar_label)                 

  plots['positions'] = ax.plot(positions[:,0],
                               positions[:,1],
                               positions[:,2],
                               c='g')

  # start animation
  anim = animation.FuncAnimation(fig,
                                 animate_plot,
                                 frames=len(plot_data),
                                 interval=1,
                                 blit=False,
                                 repeat=False)

  plt.show()
