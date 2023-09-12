
outdir = '/home/pi/Desktop/temp/coloradar';
datadir = '/media/pi/BackupPlus/jhuai/data/coloradar/rosbags';
folders = dir([outdir, '/*']);
names = {folders.name};
places = names(3:length(names) - 2);

% failed seqs of fastlioslam
places = {'arpg_lab_run4', ...
'ec_hallways_run1', ...
'edgar_army_run1', ...
'edgar_army_run4', ...
'edgar_classroom_run0', ...
'edgar_classroom_run1', ...
'edgar_classroom_run2', ...
'edgar_classroom_run3', ...
'edgar_classroom_run4', ...
'edgar_classroom_run5'};

close all;
for i=1:length(places)
    place = places{i};
    a = readmatrix([outdir, '/', place, '/scan_states.txt']);
    b = readmatrix([datadir, '/', place, '_gt.txt']);
    figure;
    plot3(a(:, 2), a(:, 3), a(:, 4), 'r'); hold on;
    plot3(b(:, 2), b(:, 3), b(:, 4), 'g'); hold on;
    axis equal; grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    legend('estimated', 'gt');
    title(place, 'Interpreter','none');
end

