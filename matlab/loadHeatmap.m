
function [heatmap_time, heatmap_intensity, heatmap_range_rate] = loadHeatmap(heatmap_data_filename, heatmap_time_filename, frame_index, heatmap_params)
   %% Read the heatmap file
   %  =====================
   num_range_bins       = heatmap_params(1);
   num_elevation_bins   = heatmap_params(2);
   num_azimuth_bins     = heatmap_params(3);
   % disp(' ')
   % disp(['opening heatmap file: ',heatmap_data_filename,'...']);
   
   % Read the binary file as a string of 16 bit integers
   
   % open the file
   fid   = fopen(heatmap_data_filename,'r');
   [input_heatmap_data, num_cnt] = fread(fid,Inf,'single'); % 32 bit float
   fclose(fid);
   
   % There should be two value per heatmap location: intensity & range rate
   expected_num_samples = 2*(num_range_bins * num_elevation_bins * num_azimuth_bins);
   assert(num_cnt == expected_num_samples, 'Number of samples read does not match expected number of samples');
   % disp(['Expected number of samples: ',num2str(expected_num_samples)]);
   % disp(['Read number of samples:     ',num2str(num_cnt)]);
   
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

   num_intensity_nans = sum(isnan(heatmap_intensity(:)));
   num_rate_nans = sum(isnan(heatmap_range_rate(:)));
   assert(num_intensity_nans == 0, 'NaNs found in intensity');
   assert(num_rate_nans == 0, 'NaNs found in range rate');
   %% Read the ADC sample time (seconds since 1-Jan-1970)
   
   time_fid                = fopen(heatmap_time_filename,'r');
   time_stamp_all_frames   = fscanf(time_fid,'%f');
   fclose(time_fid);
   heatmap_time            = time_stamp_all_frames(frame_index + 1);

   rrm = max(max(max(heatmap_range_rate)));
   rrn = min(min(min(heatmap_range_rate)));
   if rrm < 0.1 && rrn < 0.1
      heatmap_range_rate = heatmap_range_rate * 1000;
      disp(['scale up heatmap range rate whose max ', num2str(rrm), ' min ', num2str(rrn)]);
   end
end

