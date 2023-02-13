

function [adc_time, ADC_data_real_value, ADC_data_imag_value] = loadAdcFrame(adc_data_filename, adc_time_filename, frame_index, data_params)
   %% Read the ADC Samples
   %  ====================
   num_Tx = data_params(1);
   num_Rx = data_params(2);
   num_chirp = data_params(3);
   num_ADC = data_params(4);
   % disp(' ')
   % disp(['opening ADC sample file: ',adc_data_filename,'...']);
   
   % Read the binary file as a string of 16 bit integers
   
   % open the file
   fid   = fopen(adc_data_filename,'r');
   [input_data, num_cnt] = fread(fid,Inf,'int16');
   fclose(fid);
   
   % There should be num_Rx * num_Tx * num_ADC * num_chirp = real samples
   % There should be num_Rx * num_Tx * num_ADC * num_chirp = imag samples
   
   expected_num_samples = 2*(num_Rx * num_Tx * num_ADC * num_chirp);
   assert(num_cnt == expected_num_samples, 'Number of samples read does not match expected number of samples');
   % disp(['Expected number of samples: ',num2str(expected_num_samples)]);
   % disp(['Read number of samples:     ',num2str(num_cnt)]);
   
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
   adc_time                = time_stamp_all_frames(frame_index + 1);
   fclose(time_fid);
end

