% program = 'main_read_single_chip_radar_sensor_dataset'
% updated = '16-January-2021'
% author  = Christopher Williams

% This routine reads the binary files in the single chip radar sensor dataset.

% **********************************************************************

%% Define User values

source_dir              = '../raw_data/12_21_2020_arpg_lab_run0/';
device_name             = 'single_chip/';
adc_data_path           = 'adc_samples/data/';
adc_data_suffix         = '.bin';
adc_time_path           = 'adc_samples/';
adc_time_suffix         = '.txt';

heatmap_data_path       = 'heatmaps/data/';
heatmap_data_suffix     = '.bin';
heatmap_time_path       = 'heatmaps/';
heatmap_time_suffix     = '.txt';

pointcloud_data_path    = 'pointclouds/data/';
pointcloud_data_suffix  = '.bin';
pointcloud_time_path    = 'pointclouds/';
pointcloud_time_suffix  = '.txt';

%% Define cascade radar sensor parameters (from calib.zip)

% Define the waveform configuration (from waveform_cfg.txt file)
num_Tx      = 3;
num_Rx      = 4;
num_chirp   = 128;
num_ADC     = 128;
data_parm   = [num_Tx, num_Rx, num_chirp, num_ADC];

% Define the heatmap configuration (from the heatmap_cfg.txt file)
num_range_bins       = 64;
num_elevation_bins   = 16;
num_azimuth_bins     = 64;
heatmap_parm         = [num_range_bins, num_elevation_bins, num_azimuth_bins];
% for plotting:
range_bin_width      = 0.124905712903;
azimuth_bins         = [-1.33186280727 -1.22482573986 -1.14304924011 -1.07388353348 -1.01260471344 -0.956831276417 -0.905163228512 -0.85669451952 -0.810798704624 -0.767021000385 -0.725018560886 -0.684525132179 -0.645328640938 -0.607256829739 -0.570167124271 -0.533939659595 -0.498472452164 -0.463677436113 -0.42947781086 -0.395805954933 -0.362601518631 -0.329810380936 -0.29738342762 -0.265275686979 -0.233445748687 -0.201855003834 -0.1704672575 -0.139248266816 -0.108165338635 -0.0771870091558 -0.0462827570736 -0.0154226897284 0.0154226897284 0.0462827570736 0.0771870091558 0.108165338635 0.139248266816 0.1704672575 0.201855003834 0.233445748687 0.265275686979 0.29738342762 0.329810380936 0.362601518631 0.395805954933 0.42947781086 0.463677436113 0.498472452164 0.533939659595 0.570167124271 0.607256829739 0.645328640938 0.684525132179 0.725018560886 0.767021000385 0.810798704624 0.85669451952 0.905163228512 0.956831276417 1.01260471344 1.07388353348 1.14304924011 1.22482573986 1.33186280727];
elevation_bins       = [-1.18189096451 -0.930549025536 -0.74581605196 -0.588597178459 -0.446507632732 -0.313554286957 -0.18613794446 -0.0617275051773 0.0617275051773 0.18613794446 0.313554286957 0.446507632732 0.588597178459 0.74581605196 0.930549025536 1.18189096451];

%% Define frames to read

% process individual frames
for frame_index    = 200:201
      
   %% Define the full filenames
   
   adc_data_filename          = [source_dir,device_name,adc_data_path,'frame_',num2str(frame_index),adc_data_suffix];
   adc_time_filename          = [source_dir,device_name,adc_time_path,'timestamps',adc_time_suffix];
   heatmap_data_filename      = [source_dir,device_name,heatmap_data_path,'heatmap_',num2str(frame_index),heatmap_data_suffix];
   heatmap_time_filename      = [source_dir,device_name,heatmap_time_path,'timestamps',heatmap_time_suffix];
   pointcloud_data_filename   = [source_dir,device_name,pointcloud_data_path,'radar_pointcloud_',num2str(frame_index),pointcloud_data_suffix];
   pointcloud_time_filename   = [source_dir,device_name,pointcloud_time_path,'timestamps',pointcloud_time_suffix];
   
   %% Proceed if the files exist
   
   good_adc_data_filename        = exist(adc_data_filename,'file');
   good_adc_time_filename        = exist(adc_time_filename,'file');
   good_heatmap_data_filename    = exist(heatmap_data_filename,'file');
   good_heatmap_time_filename    = exist(heatmap_time_filename,'file');
   good_pointcloud_data_filename = exist(pointcloud_data_filename,'file');
   good_pointcloud_time_filename = exist(pointcloud_time_filename,'file');
   
   if((good_adc_data_filename == 2) && (good_adc_time_filename == 2) && ...
         (good_heatmap_data_filename == 2) && (good_heatmap_time_filename == 2) && ...
         (good_pointcloud_data_filename == 2) && (good_pointcloud_time_filename == 2))

      disp(' ')
      disp(['...Processing frame number: ',num2str(frame_index)]);
      
      %% Read the ADC Samples
      %  ====================
      
      disp(' ')
      disp(['opening ADC sample file: ',adc_data_filename,'...']);
      
      % Read the binary file as a string of 16 bit integers
      
      % open the file
      fid   = fopen(adc_data_filename,'r');
      [input_data, num_cnt] = fread(fid,Inf,'int16');
      
      % There should be num_Rx * num_Tx * num_ADC * num_chirp = real samples
      % There should be num_Rx * num_Tx * num_ADC * num_chirp = imag samples
      
      expected_num_samples = 2*(num_Rx * num_Tx * num_ADC * num_chirp);
      disp(['Number of expected ADC samples: ',num2str(expected_num_samples)]);
      disp(['Number of ADC samples read:     ',num2str(num_cnt)]);
      
      %% Convert from serial format to matrix format
      
      % The decomposition of serial values into matrix notation:
      % i = 2(s + Ns(c + Nc(r + tNr)))
      % where
      %      t = Tx_num  (0-2)
      %      r = Rx_num  (0-3)
      %      c = chirp_num  (0-127)
      %      s = ADC sample (0-127)
      
      ADC_data_real_value  = ones(num_Tx, num_Rx, num_chirp, num_ADC) .* NaN;
      ADC_data_imag_value  = ones(num_Tx, num_Rx, num_chirp, num_ADC) .* NaN;
      
      for index_Tx = 1:num_Tx
         % get 0-ref index
         t = index_Tx - 1;
         
         for index_Rx = 1:num_Rx
            % get 0-ref index
            r = index_Rx - 1;
            
            for index_chirp = 1:num_chirp
               % get 0-ref index
               c = index_chirp - 1;
               
               for index_ADC = 1:num_ADC
                  % get 0-ref index
                  s = index_ADC - 1;
                  
                  index_real = 2*(s + num_ADC*(c + num_chirp*(r + t*num_Rx))) + 1;
                  index_range_rate = index_real + 1;
                  
                  % save the recorded value
                  ADC_data_real_value(index_Tx, index_Rx, index_chirp, index_ADC) = input_data(index_real);
                  ADC_data_imag_value(index_Tx, index_Rx, index_chirp, index_ADC) = input_data(index_range_rate);
                                    
               end % end for index_Tx
            end % end for index_Rx
         end % end for index_chirp
      end % end for index_ADC
      
      %% Read the ADC sample time (seconds since 1-Jan-1970)
      
      time_fid                = fopen(adc_time_filename,'r');
      time_stamp_all_frames   = fscanf(time_fid,'%f');
      ADC_time                = time_stamp_all_frames(frame_index);
      
      %% Read the heatmap file
      %  =====================
      
      disp(' ')
      disp(['opening heatmap file: ',heatmap_data_filename,'...']);
      
      % Read the binary file as a string of 16 bit integers
      
      % open the file
      fid   = fopen(heatmap_data_filename,'r');
      [input_heatmap_data, num_cnt] = fread(fid,Inf,'single'); % 32 bit float
      
      % There should be two value per heatmap location: intensity & range rate
      expected_num_samples = 2*(num_range_bins * num_elevation_bins * num_azimuth_bins);
      
      disp(['Number of expected heatmap samples: ',num2str(expected_num_samples)]);
      disp(['Number of heatmap samples read:     ',num2str(num_cnt)]);
      
      %% Convert from serial format to matrix format
      
      % The decomposition of serial values into matrix notation:
      % i = 2(r + Nr(a + Na(e)))
      % where indices are 0-referenced
      %      r = num_range_bins     (0-63)
      %      a = num_azimuth_bins   (0-63)
      %      e = num_elevation_bins (0-15)
      
      heatmap_intensity  = ones(num_range_bins, num_azimuth_bins, num_elevation_bins) .* NaN;
      heatmap_range_rate = ones(num_range_bins, num_azimuth_bins, num_elevation_bins) .* NaN;
      
      for index_range = 1:num_range_bins
         % get 0-ref index
         r = index_range - 1;
         
         for index_azimuth = 1:num_azimuth_bins
            % get 0-ref index
            a = index_azimuth - 1;
            
            for index_elevation = 1:num_elevation_bins
               % get 0-base index
               e = index_elevation - 1;
               
               index_intensity = 2* (r + num_range_bins * (a + num_azimuth_bins*e)) + 1;
               index_range_rate = index_intensity + 1;

               % save the recorded value
               heatmap_intensity(index_range, index_azimuth, index_elevation) = input_heatmap_data(index_intensity);
               heatmap_range_rate(index_range, index_azimuth, index_elevation) = input_heatmap_data(index_range_rate);
               
            end % end for index_elevation
         end % end for index_azimuth
      end % end for index_range

      %% Read the heatmap time (seconds since 1-Jan-1970)
      
      time_fid                = fopen(heatmap_time_filename,'r');
      time_stamp_all_frames   = fscanf(time_fid,'%f');
      heatmap_time            = time_stamp_all_frames(frame_index);
      
      %% Read the Point Cloud file
      %  =========================
      
      disp(' ')
      disp(['opening point cloud file: ',pointcloud_data_filename,'...']);
      
      % Read the binary file as a string of 16 bit integers
      
      % open the file
      fid   = fopen(pointcloud_data_filename,'r');
      [input_pointcloud_data, num_cnt] = fread(fid,Inf,'single'); % 32 bit float
      
      % There should be 5 values per point cloud (x,y,z,intensity,velocity)
      
      num_pointcloud = num_cnt / 5;
      disp(['Number of point cloud samples read: ',num2str(num_pointcloud)]);
      
      % Read the serial data
      
      % Extract the 5 values for each target
      % i = 5(r)
      % where indices are 0-referenced
      %      r = num_pointcloud     (0-#)
      
      if(num_pointcloud > 0)
         pointcloud_x            = ones(num_pointcloud,1) .* NaN;
         pointcloud_y            = ones(num_pointcloud,1) .* NaN;
         pointcloud_z            = ones(num_pointcloud,1) .* NaN;
         pointcloud_intensity    = ones(num_pointcloud,1) .* NaN;
         pointcloud_range_rate   = ones(num_pointcloud,1) .* NaN;
         
         for index = 1:num_pointcloud
            % get 0-ref index
            r = index - 1;
            
            index_x = 5 * r + 1;
            index_y = index_x + 1;
            index_z = index_x + 2;
            index_intensity   = index_x + 3;
            index_range_rate  = index_x + 4;
            
            pointcloud_x(index)           = input_pointcloud_data(index_x);
            pointcloud_y(index)           = input_pointcloud_data(index_y);
            pointcloud_z(index)           = input_pointcloud_data(index_z);
            pointcloud_intensity(index)   = input_pointcloud_data(index_intensity);
            pointcloud_range_rate(index)  = input_pointcloud_data(index_range_rate);
            
         end % end for index loop
         
      else
         pointcloud_x            = NaN;
         pointcloud_y            = NaN;
         pointcloud_z            = NaN;
         pointcloud_intensity    = NaN;
         pointcloud_range_rate   = NaN;
         
      end % end if(num_pointcloud > 0)

      %% Read the pointcloud time (seconds since 1-Jan-1970)
      
      time_fid                = fopen(pointcloud_time_filename,'r');
      time_stamp_all_frames   = fscanf(time_fid,'%f');
      pointcloud_time         = time_stamp_all_frames(frame_index);
      
      %% Process the ADC, heatmap, and point cloud data
      
      % The ADC samples, heatmap, point cloud, and timestamp data have 
      % been loaded into the MATLAB workspace. 
      
      % User defined operations are placed here...
      
   end % end if(good_filename == 2)
   
end % end for frame_index loop






