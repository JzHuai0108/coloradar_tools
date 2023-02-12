% program = 'main_read_cascade_radar_sensor_dataset'
% updated = '16-January-2021'
% author  = Christopher Williams

% This routine reads the binary files in the cascade radar sensor dataset.

% **********************************************************************

%% Define User values

source_dir           = '../raw_data/12_21_2020_arpg_lab_run0/';
device_name          = 'cascade/';
adc_data_path        = 'adc_samples/data/';
adc_data_suffix      = '.bin';
adc_time_path        = 'adc_samples/';
adc_time_suffix      = '.txt';

heatmap_path         = 'heatmaps/data/';
heatmap_suffix       = '.bin';
heatmap_time_path    = 'heatmaps/';
heatmap_time_suffix  = '.txt';

%% Define cascade radar sensor parameters (from calib.zip)

% Define the waveform configuration (from waveform_cfg.txt file)
num_Tx      = 12;
num_Rx      = 16;
num_chirp   = 16;
num_ADC     = 256;
data_parm   = [num_Tx, num_Rx, num_chirp, num_ADC];

% Define the heatmap configuration (from the heatmap_cfg.txt file)
num_range_bins       = 128;
num_elevation_bins   = 32;
num_azimuth_bins     = 128;
heatmap_parm         = [num_range_bins, num_elevation_bins, num_azimuth_bins];
% for plotting:
range_bin_width      = 0.0592943951488;
azimuth_bins         = [-1.3669834137 -1.30120515823 -1.24833345413 -1.20276021957 -1.16203439236 -1.12482368946 -1.0903083086 -1.0579417944 -1.02733767033 -0.99821138382 -0.970345854759 -0.943571031094 -0.917750835419 -0.892774283886 -0.868549585342 -0.844999492168 -0.822058618069 -0.799670696259 -0.777787148952 -0.756365478039 -0.73536837101 -0.714762806892 -0.694519400597 -0.674611747265 -0.655016183853 -0.635711312294 -0.616677582264 -0.597897350788 -0.579354345798 -0.561033666134 -0.542921543121 -0.525005221367 -0.50727301836 -0.489713847637 -0.472317516804 -0.455074489117 -0.437975734472 -0.421012848616 -0.404177844524 -0.387463241816 -0.37086185813 -0.354366987944 -0.337972193956 -0.321671336889 -0.305458545685 -0.289328247309 -0.273275017738 -0.257293701172 -0.241379350424 -0.225527092814 -0.209732279181 -0.193990409374 -0.178297072649 -0.162647992373 -0.147039011121 -0.131466031075 -0.115925058722 -0.100412160158 -0.0849234685302 -0.0694551765919 -0.0540035180748 -0.0385647527874 -0.0231351815164 -0.0077111152932 0.0077111152932 0.0231351815164 0.0385647527874 0.0540035180748 0.0694551765919 0.0849234685302 0.100412160158 0.115925058722 0.131466031075 0.147039011121 0.162647992373 0.178297072649 0.193990409374 0.209732279181 0.225527092814 0.241379350424 0.257293701172 0.273275017738 0.289328247309 0.305458545685 0.321671336889 0.337972193956 0.354366987944 0.37086185813 0.387463241816 0.404177844524 0.421012848616 0.437975734472 0.455074489117 0.472317516804 0.489713847637 0.50727301836 0.525005221367 0.542921543121 0.561033666134 0.579354345798 0.597897350788 0.616677582264 0.635711312294 0.655016183853 0.674611747265 0.694519400597 0.714762806892 0.73536837101 0.756365478039 0.777787148952 0.799670696259 0.822058618069 0.844999492168 0.868549585342 0.892774283886 0.917750835419 0.943571031094 0.970345854759 0.99821138382 1.02733767033 1.0579417944 1.0903083086 1.12482368946 1.16203439236 1.20276021957 1.24833345413 1.30120515823 1.3669834137];
elevation_bins       = [-1.27362585068 -1.10726797581 -0.98413258791 -0.880573093891 -0.788668692112 -0.704597592354 -0.626161694527 -0.551952362061 -0.480995953083 -0.412579834461 -0.346157461405 -0.281292319298 -0.21762278676 -0.154838755727 -0.0926650241017 -0.0308490488678 0.0308490488678 0.0926650241017 0.154838755727 0.21762278676 0.281292319298 0.346157461405 0.412579834461 0.480995953083 0.551952362061 0.626161694527 0.704597592354 0.788668692112 0.880573093891 0.98413258791 1.10726797581 1.27362585068];

%% Define frames to read

% process individual frames
for frame_index    = 200:201
      
   %% Define the full filenames
   
   adc_data_filename       = [source_dir,device_name,adc_data_path,'frame_',num2str(frame_index),adc_data_suffix];
   adc_time_filename       = [source_dir,device_name,adc_time_path,'timestamps',adc_time_suffix];
   heatmap_data_filename   = [source_dir,device_name,heatmap_path,'heatmap_',num2str(frame_index),heatmap_suffix];
   heatmap_time_filename   = [source_dir,device_name,heatmap_time_path,'timestamps',heatmap_time_suffix];
   
   %% Proceed if the files exist
   
   good_adc_data_filename     = exist(adc_data_filename,'file');
   good_adc_time_filename     = exist(adc_time_filename,'file');
   good_heatmap_data_filename = exist(heatmap_data_filename,'file');
   good_heatmap_time_filename = exist(heatmap_time_filename,'file');
   
   if((good_adc_data_filename == 2) && (good_adc_time_filename == 2) && ...
         (good_heatmap_data_filename == 2) && (good_heatmap_time_filename == 2))
      
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
      disp(['Expected number of samples: ',num2str(expected_num_samples)]);
      disp(['Read number of samples:     ',num2str(num_cnt)]);
      
      %% Convert from serial format to matrix format
      
      % The decomposition of serial values into matrix notation:
      % i = 2(s + Ns(c + Nc(r + tNr)))
      % where
      %      t = Tx_num  (0-11)
      %      r = Rx_num  (0-15)
      %      c = chirp_num  (0-15)
      %      s = ADC sample (0-256)
      
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
      adc_time                = time_stamp_all_frames(frame_index);
      
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
      
      disp(['Expected number of samples: ',num2str(expected_num_samples)]);
      disp(['Read number of samples:     ',num2str(num_cnt)]);
      
      %% Convert from serial format to matrix format
      
      % The decomposition of serial values into matrix notation:
      % i = 2(r + Nr(a + Na(e)))
      % where indices are 0-referenced
      %      r = num_range_bins     (0-127)
      %      a = num_azimuth_bins   (0-127)
      %      e = num_elevation_bins (0-31)
      
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

      %% Read the ADC sample time (seconds since 1-Jan-1970)
      
      time_fid                = fopen(heatmap_time_filename,'r');
      time_stamp_all_frames   = fscanf(time_fid,'%f');
      heatmap_time            = time_stamp_all_frames(frame_index);

      %% Process the ADC, time, and heatmap data
      
      % The ADC samples, timestamp, and heatmap data have been loaded into
      % the MATLAB workspace. 
      
      % User defined operations are placed here...
      
   end % end if(good_filename == 2)
   
end % end for frame_index loop






