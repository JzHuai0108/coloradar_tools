#!/usr/bin/env python2
import os
import argparse
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation

def extract(bagfile, pose_topic, msg_type, W0_T_W, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('# timestamp W0_p_base_x W0_p_base_y W0_p_base_z W0_q_base_x W0_q_base_y W0_q_base_z '
            'W0_q_base_w W0_v_base_x W0_v_base_y W0_v_base_z\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
            if msg_type == "Odometry":
                W_R_B = Rotation.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                W_p_B = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
                if W0_T_W is None:
                    W0_R_B = W_R_B
                    W0_p_B = W_p_B
                else:
                    W0_R_B = W0_T_W[0] * W_R_B
                    W0_p_B = W0_T_W[0].apply(W_p_B) + W0_T_W[1]
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.8f %.8f %.8f\n' %
                        (msg.header.stamp.to_sec(),
                         W0_p_B[0], W0_p_B[1], W0_p_B[2],
                         W0_R_B.as_quat()[0], W0_R_B.as_quat()[1], W0_R_B.as_quat()[2], W0_R_B.as_quat()[3],
                         msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z))
            else:
                assert False, "Unknown message type"
            n += 1
    print('wrote ' + str(n) + ' messages to the file: ' + out_filename)


def extract_bag(bagfile, topic, W0_T_W_txt, out_filename):
    print('Extract and transform poses from bag '+ bagfile +' in topic ' + topic)
    if W0_T_W_txt:
        print('Transform poses with ' + W0_T_W_txt)
        data = np.loadtxt(W0_T_W_txt)
        W0_T_W = data.reshape((3, 4))
        W0_T_W = (Rotation.from_matrix(W0_T_W[:, :3]), W0_T_W[:, 3])
    else:
        print('No transformation')
        W0_T_W = None
    extract(bagfile, topic, "Odometry", W0_T_W, out_filename)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts messages from bagfile and apply transformations.
    ''')
    parser.add_argument('bagdir', help='Bag directory')
    parser.add_argument('--topic', default='/lidar_ground_truth', help='Topic')
    parser.add_argument('--W0_T_W_dir', default=None, help='Directory containing transformation matrix from W to W0')
    args = parser.parse_args()

    # find all bags under bagdir
    bagfiles = []
    seqnames = ['edgar_classroom_run', 'ec_hallways_run', 'arpg_lab_run',
                'outdoors_run', 'aspen_run', 'edgar_army_run'] # , 'longboard_run'] we cannot align these longboard seqs
    for name in seqnames:
        files = []
        for i in range(15):
            fn = os.path.join(args.bagdir, name + str(i) + '.bag')
            if os.path.isfile(fn):
                files.append(fn)
        bagfiles.append(files)

    Tfiles = []
    for name in seqnames:
        files = []
        for i in range(15):
            fn = os.path.join(args.W0_T_W_dir, name + str(i), 'W0_T_Wi.txt')
            if os.path.isfile(fn):
                files.append(fn)
        Tfiles.append(files)

    for i in range(len(bagfiles)):
        assert len(bagfiles[i]) == len(Tfiles[i]) + 1, "Number of bag files and transformation files do not match"
        for j, bagfile in enumerate(bagfiles[i]):
            if j > 0:
                assert os.path.basename(bagfile)[:-4] == os.path.basename(os.path.dirname(Tfiles[i][j-1])), \
                    "Bag file and transformation file do not match " + bagfile + " " + Tfiles[i][j-1]

    for i in range(len(bagfiles)):
        for j, bagfile in enumerate(bagfiles[i]):
            out_fn = bagfile[:-4] + '_gt.txt'
            if j == 0:
                continue
                extract_bag(bagfile, args.topic, None, out_fn)
            else:
                extract_bag(bagfile, args.topic, Tfiles[i][j-1], out_fn)
