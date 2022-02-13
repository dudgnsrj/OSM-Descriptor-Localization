%{
Nr.     Sequence name     Start   End
---------------------------------------
00: 2011_10_03_drive_0027 000000 004540
02: 2011_10_03_drive_0034 000000 004660
05: 2011_09_30_drive_0018 000000 002760
06: 2011_09_30_drive_0020 000000 001100
07: 2011_09_30_drive_0027 000000 001100
08: 2011_09_30_drive_0028 001100 005170
09: 2011_09_30_drive_0033 000000 001590
10: 2011_09_30_drive_0034 000000 001200

Same Places:
(00 & 07), (05 & 06), (09 & 10)
%}

clear;
close all;

%% Params
num_bin = 360;
N = 200; % Top N

%% File name
pc_descriptor_rot_inv_path = '../data/kitti00_rotinv_lidar_descriptor.csv';
pc_descriptor_path = '../data/kitti00_lidar_descriptor.csv';
pc_pose_path = ; % path to kitti oxts

osm_descriptor_rot_inv_path = '../data/kitti00_rotinv_osm_descriptor.csv';
osm_descriptor_path = '../data/kitti00_osm_descriptor.csv';
osm_pose_path = '../data/kitti00_osm_pose.csv';

%% Load pose data
files = dir(pc_pose_path);
files(1:2) = [];
% files(1:1099) = []; % for sequence 08, use this (and comment out above)
files = {files(:).name};

pc_poses = [];
for i = 1:length(files)
    filename = files{i};
    current_path = fullfile(pc_pose_path, filename);
    
    raw_data = importdata(current_path);
    pc_lat = raw_data(1); pc_lon = raw_data(2);
    pc_poses = [pc_poses; pc_lat, pc_lon];
end

osm_poses = csvread(osm_pose_path);

[pc_x, pc_y] = deg2utm(pc_poses(:,1), pc_poses(:,2));
osm_x = osm_poses(:,1);
osm_y = osm_poses(:,2);

pc_descriptor_rot_inv = csvread(pc_descriptor_rot_inv_path);
osm_descriptor_rot_inv = csvread(osm_descriptor_rot_inv_path);

pc_descriptor_list = csvread(pc_descriptor_path);
osm_descriptor_list = csvread(osm_descriptor_path);

%% Rotation invariant descriptor matching
topN_list = zeros(size(pc_descriptor_rot_inv,1), N);
matched_pair = zeros(size(pc_descriptor_rot_inv,1), 3);
for i = 1:size(pc_descriptor_rot_inv,1)

    if rem(i,100) == 0
        disp([num2str(i), '/', num2str(size(pc_descriptor_rot_inv,1))])
    end
    
    min_diff = inf;
    pc_descriptor = pc_descriptor_rot_inv(i,:);

    for j = 1:size(osm_descriptor_rot_inv,1)
        osm_descriptor = osm_descriptor_rot_inv(j,:);
        diff = sum(abs(pc_descriptor - osm_descriptor));
        diff_list(j) = diff;

        if diff < min_diff
            min_diff = min(diff, min_diff);
            match = j;
        end
          
    end
    
    matched_pair(i,:) = [i, match, min_diff];
    [B, I] = mink(diff_list, N);
    topN_list(i,:) = I;

end

result_N = zeros(length(matched_pair),N);
for i = 1:length(topN_list)
    topN_distances = sqrt((osm_x(topN_list(i,:)) - pc_x(matched_pair(i,1))).^2 + (osm_y(topN_list(i,:)) - pc_y(matched_pair(i,1))).^2);
    result_N(i,:) = (topN_distances < 5);
end

top_N_accuracy = zeros(1,N);
for n = 1:N
    top_N = result_N(:,1:n);
    top_N_accuracy(n) = sum(sum(top_N,2) ~= 0)/length(matched_pair);
end

disp('Stage 1 finished!')

%% compare_full_descriptor
matched_pair_full = zeros(length(pc_descriptor_list), 3);
topN_list_full = zeros(size(pc_descriptor_list,1), N);

for i = 1:length(pc_descriptor_list)
    if rem(i,100) == 0
        disp([num2str(i), '/', num2str(length(pc_descriptor_list))]);
    end
    
    pc_descriptor = pc_descriptor_list(i,:);
    
    min_diff = inf;
    diff_list = zeros(1,N);
    for j = 1:N
        osm_descriptor = osm_descriptor_list(topN_list(i,j), :);
        
        min_diff_k = inf;
        for k = 1:num_bin
            osm_descriptor_shifted = circshift(osm_descriptor,k,2);
            diff_k = sum(abs(pc_descriptor - osm_descriptor_shifted));
            
            if diff_k < min_diff_k
                min_diff_k = min(diff_k, min_diff_k);
            end
        end
        diff_list(j) = min_diff_k;

        if min_diff_k < min_diff
            min_diff = min(min_diff_k, min_diff);
            match = topN_list(i,j);
        end
    end
    
    matched_pair_full(i,:) = [i, match, min_diff];
    [B, I] = mink(diff_list, N);
    for j = 1:N
        topN_list_full(i,j) = topN_list(i,I(j));
    end
end

result_N_full = zeros(length(matched_pair_full),N);
for i = 1:length(topN_list_full)
    topN_distances = sqrt((osm_x(topN_list_full(i,:)) - pc_x(matched_pair_full(i,1))).^2 + (osm_y(topN_list_full(i,:)) - pc_y(matched_pair_full(i,1))).^2);
    result_N_full(i,:) = (topN_distances < 5);
end

top_N_accuracy_full = zeros(1,N);
for n = 1:N
    top_N_full = result_N_full(:,1:n);
    top_N_accuracy_full(n) = sum(sum(top_N_full,2) ~= 0)/length(matched_pair_full);
end

figure(1); clf; hold on; plot(pc_x, pc_y, 'black'); axis equal; title('KITTI 00', 'FontSize', 20);
scatter(pc_x(matched_pair_full((sum(result_N_full(:,1:10),2) ~= 0),1)), pc_y(matched_pair_full((sum(result_N_full(:,1:10),2) ~= 0),1)), '.', 'green');
scatter(pc_x(matched_pair_full((sum(result_N_full(:,1),2) ~= 0),1)), pc_y(matched_pair_full((sum(result_N_full(:,1),2) ~= 0),1)), 100, '.', 'blue');

figure(2); clf; hold on; plot(top_N_accuracy(1:10), 'k--');
plot(top_N_accuracy_full(1:10), 'k');
title("KITTI 00", 'FontSize', 24);
xlim([1 10]); ylim([0 1]);
xlabel("top N", 'FontSize', 16); ylabel("accuracy", 'FontSize', 16);
ax = gca; ax.FontSize = 16;
legend({'1 stage', '2 stages'}, 'Location', 'southeast')
grid on; grid minor

disp(['Top  1', ' Accuracy: ', num2str(100 * top_N_accuracy_full(1)), '%'])
disp(['Top  5', ' Accuracy: ', num2str(100 * top_N_accuracy_full(5)), '%'])
disp(['Top 10', ' Accuracy: ', num2str(100 * top_N_accuracy_full(10)), '%'])