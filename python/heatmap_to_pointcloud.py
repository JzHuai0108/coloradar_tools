

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
  parser.add_argument('-hm', '--plot_heatmap', type=str, default='false', help='true to plot radar heatmaps, false to plot pointclouds')
  parser.add_argument('-sc', '--single_chip', type=str, default='true', help='true to plot single chip data, false to plot cascade data')
  args = parser.parse_args()

  # for now, we use a global thresholding to get the point clouds,
  # in the future, we may use a CFAR (constant false alarm rate) target detector to convert the heatmaps to point clouds.
  # reference implementations are in
  # 1. https://github.com/ati-ozgur/RmSAT-CFAR
  # 2. https://github.com/tooth2/2D-CFAR
  # 3. https://github.com/sunsided/SFND_Radar_2D_CFAR
  # 4. https://github.com/tarekarahman/CFAR-Detection-for-Doppler-Radar
  # 5. maybe the TI processor sdk for cfar on adc samples.
  # TODO(binliang)
  # load heatmaps for cascaded data
  # compute some statistics, e.g., intensity histogram to choose the proper threshold
  # open bagfile in appending mode
  # for each heatmap
  #   convert to point cloud, refer to plot_pointclouds.py line 159-line 178.
  #
  #   write to the rosbag
  # close bag file


