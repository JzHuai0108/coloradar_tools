% program = 'main_read_lidar_point_cloud_dataset'
% updated = '16-January-2021'
% author  = Christopher Williams

% This routine reads the binary files in the single chip radar sensor dataset.

% **********************************************************************

%% Define User values

source_dir              = '../raw_data/12_21_2020_arpg_lab_run0/';
device_name             = 'lidar/';

pointcloud_data_path    = 'pointclouds/';
pointcloud_data_suffix  = '.bin';
pointcloud_time_path    = '';
pointcloud_time_suffix  = '.txt';

%% Define frames to read

% process individual frames
for frame_index    = 200:201
      
   %% Define the full filenames
   
   pointcloud_data_filename   = [source_dir,device_name,pointcloud_data_path,'lidar_pointcloud_',num2str(frame_index),pointcloud_data_suffix];
   pointcloud_time_filename   = [source_dir,device_name,pointcloud_time_path,'timestamps',pointcloud_time_suffix];
   
   %% Proceed if the files exist
   
   good_pointcloud_data_filename = exist(pointcloud_data_filename,'file');
   good_pointcloud_time_filename = exist(pointcloud_time_filename,'file');
   
   if((good_pointcloud_data_filename == 2) && (good_pointcloud_time_filename == 2))

      disp(' ')
      disp(['...Processing frame number: ',num2str(frame_index)]);
      
      %% Read the Point Cloud file
      %  =========================
      
      disp(' ')
      disp(['opening point cloud file: ',pointcloud_data_filename,'...']);
            
      % open the file
      fid   = fopen(pointcloud_data_filename,'r');
      [input_pointcloud_data, num_cnt] = fread(fid,Inf,'single'); % 32 bit float
      
      % There should be 4 values per target: (x,y,z,intensity)
            
      num_pointcloud = num_cnt / 4;
      disp(['Number of point cloud samples read: ',num2str(num_pointcloud)]);
      
      % Read the serial data
      
      % Extract the 4 values for each target
      % i = 4(r)
      % where indices are 0-referenced
      %      r = num_pointcloud     (0-#)
      
      if(num_pointcloud > 0)
         pointcloud_x            = ones(num_pointcloud,1) .* NaN;
         pointcloud_y            = ones(num_pointcloud,1) .* NaN;
         pointcloud_z            = ones(num_pointcloud,1) .* NaN;
         pointcloud_intensity    = ones(num_pointcloud,1) .* NaN;
         
         for index = 1:num_pointcloud
            % get 0-ref index
            r = index - 1;
            
            index_x = (4 * r) + 1;
            index_y = (4 * r) + 2;
            index_z = (4 * r) + 3;
            index_intensity   = (4 * r) + 4;
            
            pointcloud_x(index)           = input_pointcloud_data(index_x);
            pointcloud_y(index)           = input_pointcloud_data(index_y);
            pointcloud_z(index)           = input_pointcloud_data(index_z);
            pointcloud_intensity(index)   = input_pointcloud_data(index_intensity);
            
         end % end for index loop
         
      else
         pointcloud_x            = NaN;
         pointcloud_y            = NaN;
         pointcloud_z            = NaN;
         pointcloud_intensity    = NaN;
         
      end % end if(num_pointcloud > 0)

      %% Read the pointcloud time (seconds since 1-Jan-1970)
      
      time_fid                = fopen(pointcloud_time_filename,'r');
      time_stamp_all_frames   = fscanf(time_fid,'%f');
      pointcloud_time         = time_stamp_all_frames(frame_index);
      
      %% Process the liadar point cloud data
      
      % The lidar point cloud and timestamp data have 
      % been loaded into the MATLAB workspace. 
      
      % User defined operations are placed here...
      
   end % end if(good_filename == 2)
   
end % end for frame_index loop






