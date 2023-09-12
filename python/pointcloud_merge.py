# -*-coding:utf-8-*-
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
import rosbag
import sensor_msgs.point_cloud2 as pc2
import argparse
import open3d as o3d
import glob
import os
from tqdm import tqdm

def mergemap(args, base_T_lidar, bag_file):
    groud_truth_path = bag_file.split(".")[0] + '_gt.txt'

    bag = rosbag.Bag(bag_file, "r")
    bag_data = bag.read_messages(args.lidartopic)

    sampled_pcs = []
    lidar_times = []
    count = 0
    for topic, msg, t in bag_data:
        if count % args.node_skip == 0:
            lidar = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z', 'intensity'))
            points = np.array(list(lidar))
            sampled_pcs.append(points)
            lidar_times.append(msg.header.stamp.to_sec())
        count += 1

    # load groudtruth
    gt_times, gt_poses = load_poses(groud_truth_path)
    sample_gt_poses = []
    # for each lidar time, find the gt poses to the left and right of it
    gt_start_index = np.abs(np.array(gt_times) - lidar_times[0]).argmin()
    for t in lidar_times:
        while gt_start_index < len(gt_times) and gt_times[gt_start_index] < t:
            gt_start_index += 1
        if gt_start_index == len(gt_times):
            print('Append the end gt pose at {} for query at {}'.format(gt_times[-1], t))
            sample_gt_poses.append(gt_poses[gt_start_index - 1])
            continue
        if gt_start_index == 0:
            print('Append the start gt pose at {} for query at {}'.format(gt_times[0], t))
            sample_gt_poses.append(gt_poses[0])
            continue

        left = gt_start_index - 1
        right = gt_start_index

        # linearly interpolate the lidar pose
        ratio = (t - gt_times[left]) / (gt_times[right] - gt_times[left])
        pos = gt_poses[left][:3, 3] * (1 - ratio) + gt_poses[right][:3, 3] * ratio
        key_times = [gt_times[left], gt_times[right]]
        key_rots = Rotation.from_matrix([gt_poses[left][:3, :3], gt_poses[right][:3, :3]])
        slerp = Slerp(key_times, key_rots)
        rot = slerp([t]).as_matrix()[0]
        pose = np.eye(4)
        pose[:3, :3] = rot
        pose[:3, 3] = pos
        sample_gt_poses.append(pose)

    # merge points in World cood
    num_allpoints = 0
    for points in sampled_pcs:
        num_allpoints += points.shape[0]
    print("num_allpoints: {} for bag {}".format(num_allpoints, bag_file))
    np_xyz_all = np.empty([num_allpoints, 3])
    np_intensity_all = np.empty([num_allpoints, 1])
    curr_count = 0

    for idx, points in enumerate(sampled_pcs):
        temp_gt_pose = sample_gt_poses[idx]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd_bs = pcd.transform(base_T_lidar)
        pcd_global = pcd_bs.transform(temp_gt_pose)
        xyz_global = np.asarray(pcd_global.points)
        scan_intensity = np.asarray(points[:, 3]).reshape(-1, 1)

        np_xyz_all[curr_count:curr_count + xyz_global.shape[0], :] = xyz_global
        np_intensity_all[curr_count:curr_count + xyz_global.shape[0], :] = scan_intensity
        curr_count = curr_count + xyz_global.shape[0]

    # down sample and save pcd
    np_xyz_all = np_xyz_all[0:curr_count, :]
    pcd_all = o3d.geometry.PointCloud()
    pcd_all.points = o3d.utility.Vector3dVector(np_xyz_all)
    pcd_downsampled = pcd_all.voxel_down_sample(voxel_size=args.down_voxel_size)
    print('downsampled from {} to {}'.format(np_xyz_all.shape[0], np.asarray(pcd_downsampled.points).shape[0]))
    return pcd_downsampled, sample_gt_poses

def load_poses(poses_path):
    poses = []
    gt_times =[]
    with open(poses_path, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if i ==0:
                continue
            time = np.fromstring(line, sep=' ')[0]
            temp = np.fromstring(line, dtype=np.float32, sep=' ')
            xyz = temp[1:4].reshape(3, 1)
            quaternion = temp[4:8]
            r = Rotation.from_quat(quaternion)
            rotation_matrix = r.as_matrix()
            pose = np.hstack((rotation_matrix, xyz))
            pose = np.vstack((pose, [0, 0, 0, 1]))
            poses.append(pose)
            gt_times.append(time)
    return gt_times, poses


def read_calib(calib_path):
    with open(calib_path, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if i==0:
                xyz = np.fromstring(line, sep=' ').reshape(3, 1)
            if i==1:
                quaternion = np.fromstring(line, sep=' ')
                r = Rotation.from_quat(quaternion)
                rotation_matrix = r.as_matrix()
        calib = np.hstack((rotation_matrix, xyz))
        calib = np.vstack((calib, [0, 0, 0, 1]))
    return calib


def load_trans(trans_path):
    with open(trans_path, 'r') as f:
        lines = f.readlines()
        pose = []
        for i, line in enumerate(lines):
            if i==0:
                db_id = int(((line.split(' ')[0]).split('.')[0]).split(':')[-1])
                query_id = int(((line.split(' ')[1]).split('.')[0]).split(':')[-1])
            else:
                pose_i = np.fromstring(line, dtype=np.float32, sep=' ')
                pose.append(pose_i)
        pose = np.asarray(pose)
    return db_id, query_id, pose


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='turn list of rosbags into ColoRadar dasatet format')
    parser.add_argument('datapath', type=str, default='/media/cyw/CYW-ZX2/coloradar/data/',
                        help='The folder containing all bag files')
    parser.add_argument('--savepath', type=str, default='/media/cyw/CYW-ZX2/coloradar/mergemap/',
                        help='the overarching output path to save results for all seqs. It is the output path of align_coloradar_seqs.')
    parser.add_argument('--lidartopic', type=str, default='/os1_cloud_node/points',
                        help='lidartopic')
    parser.add_argument('--bs_to_lidar_path', type=str,
                        default='/media/cyw/CYW-ZX2/coloradar/calib/transforms/base_to_lidar.txt')

    parser.add_argument('--node_skip', type=int, default=10)
    parser.add_argument('--down_voxel_size', type=float, default=0.1)
    args = parser.parse_args()

    base_T_lidar = read_calib(args.bs_to_lidar_path)

    if not os.path.exists(args.savepath):
        os.makedirs(args.savepath)

    database_gt = []
    query_gt = []

    groups = [['edgar_classroom_run0', 'edgar_classroom_run1', 'edgar_classroom_run2', 'edgar_classroom_run3',
               'edgar_classroom_run4', 'edgar_classroom_run5'],
              ['ec_hallways_run0', 'ec_hallways_run1', 'ec_hallways_run2', 'ec_hallways_run3', 'ec_hallways_run4'],
              ['arpg_lab_run0', 'arpg_lab_run1', 'arpg_lab_run2', 'arpg_lab_run3', 'arpg_lab_run4'],
              ['outdoors_run0', 'outdoors_run1', 'outdoors_run2', 'outdoors_run3', 'outdoors_run4',
               'outdoors_run5', 'outdoors_run6', 'outdoors_run7', 'outdoors_run8', 'outdoors_run9'],
              ['aspen_run0', 'aspen_run1', 'aspen_run2', 'aspen_run3', 'aspen_run4', 'aspen_run5',
               'aspen_run6', 'aspen_run7', 'aspen_run8', 'aspen_run9', 'aspen_run10', 'aspen_run11'],
              ['edgar_army_run0', 'edgar_army_run1', 'edgar_army_run2', 'edgar_army_run3', 'edgar_army_run4',
               'edgar_army_run5'],
              ['longboard_run0', 'longboard_run1', 'longboard_run2', 'longboard_run3', 'longboard_run4',
               'longboard_run5', 'longboard_run6', 'longboard_run7']]
    bag_files = []
    for group in groups:
        groupbags = []
        for seq in group:
            groupbags.append(os.path.join(args.datapath, seq + '.bag'))
        bag_files.append(groupbags)
    # print bagfiles in each group
    for g, group in enumerate(bag_files):
        for b, bag_file in enumerate(group):
            print("{}:{}:{}".format(g, b, bag_file))

    for group in tqdm(bag_files):
        for i, bag_file in enumerate(group):
            seq_name = os.path.basename(bag_file).split(".")[0]
            pcd_downsampled, sampled_gt_pose = mergemap(args, base_T_lidar, bag_file)
            # Save sampled_gt_pose to txt file
            with open(os.path.join(args.savepath, seq_name, 'sampled_gt.txt'), 'w') as f:
                for pose in sampled_gt_pose:
                    pose_str = ' '.join(str(x) for x in pose.flatten())
                    f.write(pose_str + '\n')

            # save merged pointcloud
            save_pcd_path = os.path.join(args.savepath, seq_name, "mergedmap.pcd")
            o3d.io.write_point_cloud(save_pcd_path, pcd_downsampled)
            print('Saved merged point cloud to {} for {}'.format(save_pcd_path, bag_file))

