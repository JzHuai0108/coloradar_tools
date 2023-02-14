import numpy as np
import math
from scipy.spatial.transform import Rotation, Slerp


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
def get_heatmap_points(params, min_range_idx):

  # transform range-azimuth-elevation heatmap to pointcloud
  pcl = np.zeros([params['num_elevation_bins'],
                  params['num_azimuth_bins'],
                  params['num_range_bins'] - min_range_idx,
                  5])

  for range_idx in range(params['num_range_bins'] - min_range_idx):
    for az_idx in range(params['num_azimuth_bins']):
      for el_idx in range(params['num_elevation_bins']):
        pcl[el_idx,az_idx,range_idx,:3] = polar_to_cartesian(range_idx + min_range_idx, az_idx, el_idx, params)

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
