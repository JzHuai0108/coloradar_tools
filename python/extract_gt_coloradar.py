import os
import subprocess

rootdir = '/media/pi/BackupPlus/jhuai/data/coloradar/rosbags/'
topic = '/lidar_ground_truth'
type = 'Odometry'
bagtopose_script = '/home/pi/Documents/tests/vio_common/python/bag_to_pose.py'
bags = []

for subdir, dirs, files in os.walk(rootdir):
    for file in files:
        if ".bag" in file:
            filepath = rootdir + '/' + file
            bags.append(filepath)

for i in range(len(bags)):
    print('{}: {}'.format(i, bags[i]))

for i in range(len(bags)):
    subprocess.call(['python3', bagtopose_script, bags[i], topic, '--msg_type=' + type])
