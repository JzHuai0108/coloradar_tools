rootdir = '/media/jhuai/BackupPlus/jhuai/data/coloradar/rosbags/';
addpath('/media/jhuai/BackupPlus/jhuai/tools/export_fig');
groups = {{'aspen_run0.bag', ...
'aspen_run1.bag', ...
'aspen_run2.bag', ...
'aspen_run3.bag', ...
'aspen_run4.bag', ...
'aspen_run5.bag', ...
'aspen_run6.bag', ...
'aspen_run7.bag', ...
'aspen_run8.bag', ...
'aspen_run9.bag', ...
'aspen_run10.bag', ...
'aspen_run11.bag'}, ...
{'arpg_lab_run0.bag', ...
'arpg_lab_run1.bag', ...
'arpg_lab_run2.bag', ...
'arpg_lab_run3.bag', ...
'arpg_lab_run4.bag'}, ...
{'outdoors_run0.bag', ...
'outdoors_run1.bag', ...
'outdoors_run2.bag', ...
'outdoors_run3.bag', ...
'outdoors_run4.bag', ...
'outdoors_run5.bag', ...
'outdoors_run6.bag', ...
'outdoors_run7.bag', ...
'outdoors_run8.bag', ...
'outdoors_run9.bag'}, ...
{'longboard_run0.bag', ...
'longboard_run1.bag', ...
'longboard_run2.bag', ...
'longboard_run3.bag', ...
'longboard_run4.bag', ...
'longboard_run5.bag', ...
'longboard_run6.bag', ...
'longboard_run7.bag'}, ...
{'edgar_army_run1.bag', ...
'edgar_army_run2.bag', ...
'edgar_army_run3.bag', ...
'edgar_army_run4.bag', ...
'edgar_army_run5.bag'}, ...
{'ec_hallways_run0.bag', ...
'ec_hallways_run1.bag', ...
'ec_hallways_run2.bag', ...
'ec_hallways_run3.bag', ...
'ec_hallways_run4.bag'}, ...
{'edgar_classroom_run0.bag', ...
'edgar_classroom_run1.bag', ...
'edgar_classroom_run2.bag', ...
'edgar_classroom_run3.bag', ...
'edgar_classroom_run4.bag', ...
'edgar_classroom_run5.bag'}};
% aspen, we use the 0,1,2 seq for mapping,
% arpg lab, 0,1 seq for mapping
% 

c = {'r', 'g--', 'b--', 'k--', 'm--', 'c--', 'r-.', 'g-.', 'b-.', 'k-.', 'm-.', 'c-.'};
close all;
for i=1:length(groups)
    figure;
    handles = zeros(1, length(groups{i}));
    for j=1:length(groups{i})
        fn = [rootdir, groups{i}{j}(1:end-4), '_gt.txt'];
        d = readmatrix(fn);
        h = plot3(d(:, 2), d(:, 3), d(:, 4), c{j}); hold on;
        handles(j) = h;
        plot3(d(1, 2), d(1, 3), d(1, 4), 'o', 'MarkerSize', 12);
        plot3(d(end, 2), d(end, 3), d(end, 4), 's', 'MarkerSize', 12);
    end
    grid on;
    axis equal;
    legend(handles, groups{i}, 'Interpreter','none');
    title(groups{i}{1}(1:end-5), 'Interpreter','none');
    figname = [rootdir, 'gt/', groups{i}{1}(1:end-5), '.pdf'];
    export_fig(figname);
end
