function show_mergedmap_alignment()
addpath('/media/jhuai/BackupPlus/jhuai/tools/export_fig');

datadir = '/media/jhuai/BackupPlus/jhuai/results/align_coloradar';
bagdir = '/media/jhuai/BackupPlus/jhuai/data/coloradar/rosbags';
seqnames = {'edgar_classroom_run', 'ec_hallways_run', 'arpg_lab_run', ...
            'outdoors_run', 'aspen_run', 'edgar_army_run', 'longboard_run'};
refids = [0, 0, 0, 0, 0, 0, 0];
seqids = {1:15, 1:15, 1:15. 1:15, 1:15, 1:15, 1:15};

close all;

for s = 1:6 % numel(seqnames)
    seqname = seqnames{s};
    basepcd = [datadir, '/', seqname, num2str(refids(s)), '/mergedmap.pcd'];
    fixed = pcread(basepcd);
    maxNumPoints = 12;
    fixedDownsampled = pcdownsample(fixed,"nonuniformGridSample",maxNumPoints);
    fprintf('fixed points %d after downsample %d\n', size(fixed.Location, 1), size(fixedDownsampled.Location, 1));
    for i=seqids{s}
        querypcd = [datadir, '/', seqname, num2str(i), '/mergedmap.pcd'];
        if ~isfile(querypcd)
            continue;
        end
        fprintf('Showing %s\n', querypcd);
        moving = pcread(querypcd);
        queryposefile = [datadir, '/', seqname, num2str(i), '/W0_T_Wi.txt'];

        if ~isfile(queryposefile)
            fprintf('Failed to find pose file %s\n', queryposefile);
            continue;
        end

        fprintf('queryfile %s\n', queryposefile);
        tform = read_transform(queryposefile);
        points = [moving.Location, ones(size(moving.Location, 1), 1)] * tform';

        ptCloudAligned = pointCloud(points(:, 1:3));
        figure;
        pcshowpair(ptCloudAligned, fixed);
        view(0, 90);
        title([num2str(i), ' to ', num2str(refids(s)), ' ', seqname], 'Interpreter', 'none');
        figname = [bagdir, '/gt/', seqname, num2str(i), '_cloud_W', num2str(refids(s)), '.png'];
        saveas(gcf, figname);
    end
end
end

