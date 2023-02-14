function points = heatmap_to_targetpoints(heatmap_intensity, heatmap_range_rate)
% use cfar to convert the heatmap to salient target points
% heatmap_intensity: range bins x azimuth bins x elevation bins
% points Nx5, each row [x, y, z, intensity, range rate]

close all;

azimuth_bins         = [-1.3669834137 -1.30120515823 -1.24833345413 -1.20276021957 -1.16203439236 -1.12482368946 -1.0903083086 -1.0579417944 -1.02733767033 -0.99821138382 -0.970345854759 -0.943571031094 -0.917750835419 -0.892774283886 -0.868549585342 -0.844999492168 -0.822058618069 -0.799670696259 -0.777787148952 -0.756365478039 -0.73536837101 -0.714762806892 -0.694519400597 -0.674611747265 -0.655016183853 -0.635711312294 -0.616677582264 -0.597897350788 -0.579354345798 -0.561033666134 -0.542921543121 -0.525005221367 -0.50727301836 -0.489713847637 -0.472317516804 -0.455074489117 -0.437975734472 -0.421012848616 -0.404177844524 -0.387463241816 -0.37086185813 -0.354366987944 -0.337972193956 -0.321671336889 -0.305458545685 -0.289328247309 -0.273275017738 -0.257293701172 -0.241379350424 -0.225527092814 -0.209732279181 -0.193990409374 -0.178297072649 -0.162647992373 -0.147039011121 -0.131466031075 -0.115925058722 -0.100412160158 -0.0849234685302 -0.0694551765919 -0.0540035180748 -0.0385647527874 -0.0231351815164 -0.0077111152932 0.0077111152932 0.0231351815164 0.0385647527874 0.0540035180748 0.0694551765919 0.0849234685302 0.100412160158 0.115925058722 0.131466031075 0.147039011121 0.162647992373 0.178297072649 0.193990409374 0.209732279181 0.225527092814 0.241379350424 0.257293701172 0.273275017738 0.289328247309 0.305458545685 0.321671336889 0.337972193956 0.354366987944 0.37086185813 0.387463241816 0.404177844524 0.421012848616 0.437975734472 0.455074489117 0.472317516804 0.489713847637 0.50727301836 0.525005221367 0.542921543121 0.561033666134 0.579354345798 0.597897350788 0.616677582264 0.635711312294 0.655016183853 0.674611747265 0.694519400597 0.714762806892 0.73536837101 0.756365478039 0.777787148952 0.799670696259 0.822058618069 0.844999492168 0.868549585342 0.892774283886 0.917750835419 0.943571031094 0.970345854759 0.99821138382 1.02733767033 1.0579417944 1.0903083086 1.12482368946 1.16203439236 1.20276021957 1.24833345413 1.30120515823 1.3669834137];
elevation_bins       = [-1.27362585068 -1.10726797581 -0.98413258791 -0.880573093891 -0.788668692112 -0.704597592354 -0.626161694527 -0.551952362061 -0.480995953083 -0.412579834461 -0.346157461405 -0.281292319298 -0.21762278676 -0.154838755727 -0.0926650241017 -0.0308490488678 0.0308490488678 0.0926650241017 0.154838755727 0.21762278676 0.281292319298 0.346157461405 0.412579834461 0.480995953083 0.551952362061 0.626161694527 0.704597592354 0.788668692112 0.880573093891 0.98413258791 1.10726797581 1.27362585068];
range_bin_width      = 0.0592943951488;
num_range_bins       = 128;
azimuth_grid = azimuth_bins * 180 / pi;
elevation_grid = elevation_bins * 180 / pi;
rng_grid = (0:127) * range_bin_width;


for ei =16:16
close all
f1 = figure;
imagesc(azimuth_grid, rng_grid, heatmap_intensity(:, :, ei))
% img = squeeze(heatmap_range_rate(:, 64, :));
% imagesc(elevation_grid, rng_grid, img);
xlabel('Azimuth (Deg)'); ylabel('Range (m)'); title('Range Azimuth Heatmap');
movegui(f1, 'northwest');

p = 3e-3;
detector = phased.CFARDetector2D('TrainingBandSize',[2,2], ...
    'GuardBandSize',[4,4],'ProbabilityFalseAlarm',p, ...
    'ThresholdFactor','Auto','Method','CA', ...
    'CustomThresholdFactor',0.2,'ThresholdOutputPort',true);
% ThresholdFactor Custom works poorly.

Ngc = detector.GuardBandSize(2);
Ngr = detector.GuardBandSize(1);
Ntc = detector.TrainingBandSize(2);
Ntr = detector.TrainingBandSize(1);
cutidx = [];
colstart = Ntc + Ngc + 1;
colend = size(heatmap_intensity, 2) - ( Ntc + Ngc);
rowstart = Ntr + Ngr + 1;
rowend = size(heatmap_intensity, 1) - ( Ntr + Ngr);
for m = colstart:colend
    for n = rowstart:rowend
        cutidx = [cutidx,[n;m]]; % column major order
    end
end
rangeIndx = [rowstart, rowend];
azimuthIndx = [colstart, colend];
rangeLen = rowend - rowstart + 1;
azimuthLen = colend - colstart + 1;

ncutcells = size(cutidx,2);
[dets,th] = detector(heatmap_intensity, cutidx);
% detector on MxNxP or detector on MxN layers, results are almost identical

numpoints = 0;
for k=1:32
    for p=1:size(dets, 1)
        if dets(p, k) == 1
            numpoints = numpoints + 1;
        end
    end
end
disp(['Detected points ', num2str(numpoints)]);

id = 1;
points = zeros(numpoints, 5);
elevchannels = 1:32;
% elevchannels = 1:size(heatmap_intensity, 3);
for k=elevchannels
    checkNearestMat = zeros(128, 128);
    for p=1:size(dets, 1)
        if dets(p, k) == 1
            azibinid = floor((p - 1) / rangeLen) + colstart;
            rngbinid = rem(p - 1, rangeLen) + rowstart;
            checkNearestMat(rngbinid, azibinid) = 1;
        end
    end
    for c=1:128
        objdet = 0;
        for r=1:128
            if checkNearestMat(r, c) == 1
                if objdet == 1
                    checkNearestMat(r, c) = 0;
                else
                    objdet = 1;                
                    azibinid = c;
                    rngbinid = r;
                    azi = azimuth_bins(azibinid);
                    rng = (rngbinid - 1) * range_bin_width;
                    elev = elevation_bins(k);
                    ce = cos(elev);
                    points(id, 1) = rng * ce * cos(azi);
                    points(id, 2) = rng * ce * sin(azi);
                    points(id, 3) = rng * sin(elev);
                    intensity = heatmap_intensity(rngbinid, azibinid, k);
                    rangerate = heatmap_range_rate(rngbinid, azibinid, k);
                    points(id, 4) = intensity;
                    points(id, 5) = rangerate;
                    id = id + 1;

                end
            end
        end
    end
 
end
points = points(1:id, :);

mi = max(max(max(heatmap_intensity)));
ni = min(min(min(heatmap_intensity)));
% normalize intensity values
heatmap_intensity = heatmap_intensity - ni;
heatmap_intensity = heatmap_intensity / (mi - ni);
disp(['Intensity range ', num2str(ni), ' ', num2str(mi)]);
thresh = 0.2;
numbrightpoints = find(heatmap_intensity >= thresh);
points2 = zeros(length(numbrightpoints), 5);
pid = 1;
for k =1:32
    [rowids, colids] = find(heatmap_intensity(:, :, k) >= thresh);
    for i = 1:length(rowids)
        rngid = rowids(i);
        azimid = colids(i);
        azi = azimuth_bins(azimid);
        rng = (rngid - 1) * range_bin_width;
        elev = elevation_bins(k);

        points2(pid, 1) = rng * cos(elev) * cos(azi);
        points2(pid, 2) = rng * cos(elev) * sin(azi);
        points2(pid, 3) = rng * sin(elev);
        intensity = heatmap_intensity(rngid, azimid, k);
        rangerate = heatmap_range_rate(rngid, azimid, k);
        points2(pid, 4) = intensity;
        points2(pid, 5) = rangerate;
        pid = pid + 1;
    end
end


f2 = figure;
detectionMap = zeros(size(heatmap_intensity, 1), size(heatmap_intensity, 2));
detectionMap(rangeIndx(1):rangeIndx(2),azimuthIndx(1):azimuthIndx(2)) = ...
  reshape(double(dets(:, 1)),rangeLen,azimuthLen);
h = imagesc(azimuth_grid,rng_grid,detectionMap);
xlabel('Azimuth (Deg)'); ylabel('Range (m)'); title('Range Azimuth CFAR Detections');
h.Parent.YDir = 'normal';
movegui(f2, 'northeast');

f3 = figure;
detectionMap2 = zeros(size(heatmap_intensity, 1), size(heatmap_intensity, 2));
detectionMap2(rangeIndx(1):rangeIndx(2),azimuthIndx(1):azimuthIndx(2)) = ...
  reshape(double(dets(:, 32)),rangeLen,azimuthLen);
h = imagesc(azimuth_grid,rng_grid,detectionMap2);
xlabel('Azimuth (Deg)'); ylabel('Range (m)'); title('Range Azimuth CFAR Detections');
h.Parent.YDir = 'normal';
movegui(f3, 'southwest');

f4 = figure;
plot3(points(:, 1), points(:, 2), points(:, 3), 'k.');
hold on;
plot3(points2(:, 1), points2(:, 2), points2(:, 3), 'r.');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('cfar', 'threshold');
% movegui(f5, 'southeast');

pause(0.5);
end

end
