% program = 'main_read_cascade_radar_sensor_dataset'
% updated = '16-January-2021'
% author  = Christopher Williams

% This routine reads the binary files in the cascade radar sensor dataset.

% **********************************************************************

%% Define User values

source_dir           = '../2_28_2021_outdoors_run0/';
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
% get all files in folder
files = dir([source_dir,device_name,heatmap_path,'*', heatmap_suffix]);
% get the filenames
filenames = {files.name};
numheatmaps = length(filenames);
for i = 1:length(filenames)
   filenames{i} = [source_dir,device_name,heatmap_path,filenames{i}];
end
disp(['number of heatmaps ', num2str(numheatmaps)]);
disp(['first heatmap file name ', filenames{1}]);

for frame_index    = 0:numheatmaps-1
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
      
      % disp(' ')
      disp(['...Processing frame number: ',num2str(frame_index)]);
      [adc_time, ADC_data_real_value, ADC_data_imag_value] = loadAdcFrame(adc_data_filename, adc_time_filename, frame_index, data_parm);
      [heatmap_time, heatmap_intensity, heatmap_range_rate] = loadHeatmap(heatmap_data_filename, heatmap_time_filename, frame_index, heatmap_parm);

      %% Process the ADC, time, and heatmap data
      disp(['heatmap time ', num2str(heatmap_time), ' adc-heatmap time ', num2str(adc_time - heatmap_time)]);
      % The ADC samples, timestamp, and heatmap data have been loaded into
      % the MATLAB workspace. 
      
      % User defined operations are placed here...
      
   end % end if(good_filename == 2)
   
end % end for frame_index loop
