% program = 'main_read_imu_dataset'
% updated = '16-January-2021'
% author  = Christopher Williams

% This routine reads the binary files in the single chip radar sensor dataset.

% **********************************************************************

%% Define User values

source_dir        = '../raw_data/12_21_2020_arpg_lab_run0/';
device_name       = 'imu/';

imu_data_path     = '';
imu_data_suffix   = '.txt';
imu_time_path     = '';
imu_time_suffix   = '.txt';


%% Define the full filenames

imu_data_filename   = [source_dir,device_name,imu_data_path,'imu_data',imu_data_suffix];
imu_time_filename   = [source_dir,device_name,imu_time_path,'timestamps',imu_time_suffix];

%% Proceed if the files exist

good_imu_data_filename = exist(imu_data_filename,'file');
good_imu_time_filename = exist(imu_time_filename,'file');

if((good_imu_data_filename == 2) && (good_imu_time_filename == 2))
   
   %% Read the IMU file
   %  =================
   
   disp(' ')
   disp(['opening imu file: ',imu_data_filename,'...']);
   
   % open the file
   imu_fid                 = fopen(imu_data_filename,'r');
   input_imu_data          = fscanf(imu_fid,'%f');

   % There should be 6 values per line:
   % 3 accelerations & 3 angular rates
   
   num_imu  = length(input_imu_data) / 6;
   disp(['Number of imu samples read: ',num2str(num_imu)]);
   
   % Read the serial data
   
   imu_a_x        = ones(num_imu,1) .* NaN;
   imu_a_y        = ones(num_imu,1) .* NaN;
   imu_a_z        = ones(num_imu,1) .* NaN;
   imu_alpha_x    = ones(num_imu,1) .* NaN;
   imu_alpha_y    = ones(num_imu,1) .* NaN;
   imu_alpha_z    = ones(num_imu,1) .* NaN;
   
   for index = 1:num_imu
      % get 0-ref index
      r = 6*(index - 1);
      
      imu_a_x(index)     = input_imu_data(r+1);
      imu_a_y(index)     = input_imu_data(r+2);
      imu_a_z(index)     = input_imu_data(r+3);
      imu_alpha_x(index) = input_imu_data(r+4);
      imu_alpha_y(index) = input_imu_data(r+5);
      imu_alpha_z(index) = input_imu_data(r+6);
      
   end % end for r loop
   
   %% Read the imu time (seconds since 1-Jan-1970)
   
   imu_fid           = fopen(imu_time_filename,'r');
   imu_time          = fscanf(imu_fid,'%f');
   
   %% Process the imu data
   
   % The lidar point cloud and timestamp data have
   % been loaded into the MATLAB workspace.
   
   % User defined operations are placed here...
   
end % end if(good_filename == 2)
