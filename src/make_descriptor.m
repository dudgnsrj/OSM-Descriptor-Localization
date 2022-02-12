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

clear; close all;

seq_names = {'00', '02', '05', '06', '07', '08', '09', '10'};

osm = true;
lidar = true;

num_bin = 360;
sensor_range = 50;
bin_size = 5; % bin size of rotation-invariant descriptor

%% Main
for ii=1:length(seq_names)
    seq_name = seq_names{ii};
    
    disp(['KITTI sequence ', seq_name, ' start.']);
   
    %% load files
    
    % osm
    building_file_name = '../data/kitti00_buildings.geojson'; % building geojson file path
    road_file_name = '../data/kitti00_roads.geojson'; % road geojson file path
    osm_save_path = ; % OSM descriptor save path
    osm_rotinv_save_path = ; % OSM rotation-invariant descriptor save path
    
    fid = fopen(building_file_name);
    raw = fread(fid,inf);
    str = char(raw');
    fclose(fid);
    val = jsondecode(str);
    
    fid = fopen(road_file_name);
    raw = fread(fid,inf);
    str = char(raw');
    fclose(fid);
    road_val = jsondecode(str);
    
    % lidar
    pc_path = ; % Building pointcloud path (folder)
    lidar_save_path = ; % LiDAR descriptor save path 
    lidar_rotinv_save_path = ; % LiDAR rotation-invariant descriptor save path
    
    files = dir(pc_path);
    files(1:2) = [];
    files = {files(:).name};
    
    % compare
    
    disp(['KITTI sequence ', seq_name, ' file loaded.']);
    
    %% make osm descriptor
    if osm == true
        coords_x = [];
        coords_y = [];
        for i = 1:length(road_val.features)
            feature = road_val.features(i);
            geometry = feature.geometry;
            coordinates = geometry.coordinates;
            if iscell(coordinates)
                coords_out = coordinates{1,1};
            else
                if iscell(coordinates)
                    coords_out = coordinates{1,1};
                    coords_out = reshape(coordinates, [], 2);
                else
                    coords_out = reshape(coordinates, [], 2);
                end
            end
            [x_coords_out_temp, y_coords_out_temp] = deg2utm(coords_out(:,2), coords_out(:,1));
            coords_x = [coords_x; x_coords_out_temp];
            coords_y = [coords_y; y_coords_out_temp];
        end
    
        x_coords_out = {};
        y_coords_out = {};
        for i = 1:length(val.features)
            feature = val.features(i);
            geometry = feature.geometry;
            coordinates = geometry.coordinates;

            if iscell(coordinates)
                coords_out = coordinates{1,1};
                if iscell(coords_out)
                    coords_out = coords_out{1,1};
                end
            else
                coords_out = reshape(coordinates, [], 2);
            end

            [x_coords_out_temp, y_coords_out_temp] = deg2utm(coords_out(:,2), coords_out(:,1));

            x_coords_out{i} = x_coords_out_temp;
            y_coords_out{i} = y_coords_out_temp;
        end

        descriptor = zeros(length(coords_x), num_bin);
        count = 0;
        for i = 1:length(coords_x)
            count = count + 1;

            x = coords_x(i);
            y = coords_y(i);
            descriptor_i = zeros(1, num_bin);
            for j = 1:num_bin
                target_angle_orig = j*2*pi/num_bin;
                shortest_d = sensor_range;

                for k = 1:length(x_coords_out)
                    x_coords_out_temp = x_coords_out{k};
                    y_coords_out_temp = y_coords_out{k};

                    distances = (x_coords_out_temp-x).^2 + (y_coords_out_temp-y).^2;
                    if min(distances) > 3600
                        continue
                    end

                    for l = 1:length(x_coords_out_temp)
                        vertex1 = [x_coords_out_temp(rem(l-1,length(x_coords_out_temp))+1), y_coords_out_temp(rem(l-1,length(y_coords_out_temp))+1)];
                        vertex2 = [x_coords_out_temp(rem(l+1-1,length(x_coords_out_temp))+1), y_coords_out_temp(rem(l+1-1,length(y_coords_out_temp))+1)];

                        theta1 = atan2(vertex1(2)-y, vertex1(1)-x);
                        theta2 = atan2(vertex2(2)-y, vertex2(1)-x);

                        if theta1 < 0
                            theta1 = theta1 + 2*pi;
                        end
                        if theta2 < 0
                            theta2 = theta2 + 2*pi;
                        end

                        max_theta = max(theta1, theta2);
                        theta1 = theta1 + pi - max_theta;
                        theta2 = theta2 + pi - max_theta;
                        target_angle = target_angle_orig + pi - max_theta;

                        if theta1 < 0
                            theta1 = theta1 + 2*pi;
                        elseif theta1 > 2*pi
                            theta1 = theta1 - 2*pi;
                        end
                        if theta2 < 0
                            theta2 = theta2 + 2*pi;
                        elseif theta2 > 2*pi
                            theta2 = theta2 - 2*pi;
                        end
                        if target_angle < 0
                            target_angle = target_angle + 2*pi;
                        elseif target_angle > 2*pi
                            target_angle = target_angle - 2*pi;
                        end

                        if median([target_angle, theta1, theta2]) == target_angle
                            if min([target_angle, theta1, theta2]) == theta1
                                a = norm([x-vertex1(1), y-vertex1(2)]);
                                b = norm([x-vertex2(1), y-vertex2(2)]);
                                phi1 = target_angle - theta1;
                                phi2 = theta2 - target_angle;

                            else
                                a = norm([x-vertex2(1), y-vertex2(2)]);
                                b = norm([x-vertex1(1), y-vertex1(2)]);
                                phi1 = target_angle - theta2;
                                phi2 = theta1 - target_angle;
                            end

                            d = (a*b*sin(phi1+phi2)) / (a*sin(phi1) + b*sin(phi2));

                            shortest_d = min(d, shortest_d);
                        end
                    end

                end
                if shortest_d == sensor_range
                    descriptor_i(j) = 0;
                else
                    descriptor_i(j) = shortest_d;
                end
            end
            descriptor(i, :) = descriptor_i;
        end

        csvwrite(osm_save_path, descriptor);

        osm_descriptor_rot_inv = zeros(length(descriptor), sensor_range/bin_size);
        for i = 1:length(descriptor)
            range_angle_descriptor = zeros(360, sensor_range/bin_size);

            osm_descriptor = descriptor(i,:);
            for j = 1:360
                if osm_descriptor(j) ~= 0
                    range_angle_descriptor(j,ceil(osm_descriptor(j)/bin_size)) = 1;
                end
            end
            rot_inv_descriptor = sum(range_angle_descriptor);

            osm_descriptor_rot_inv(i,:) = rot_inv_descriptor;
        end

        csvwrite(osm_rotinv_save_path, osm_descriptor_rot_inv);

        disp(['KITTI sequence ', seq_name, ' OSM descriptor is generated.']);
    else
        disp('skip OSM descriptor.');
    end
    
    %% make lidar descriptor
    if lidar == true
        descriptor_list = zeros(length(files), num_bin);
        for i = 1:length(files)
            filename = files{i};
            current_path = fullfile(pc_path, filename);

            pc = pcread(current_path);

            pc = pc.Location;

            tan_list = atan2(pc(:,2), pc(:,1));
            tan_list = tan_list + (tan_list < 0)*2*pi;

            descriptor = zeros(1, num_bin);
            for j = 1:num_bin
                target_angle_orig = (j-1)*2*pi/num_bin;

                target_angle = pi;
                tan_list_temp = tan_list + pi - target_angle_orig;
                tan_list_temp = tan_list_temp + (tan_list_temp < 0)*2*pi;
                tan_list_temp = tan_list_temp - (tan_list_temp > 2*pi)*2*pi;
                target_points = pc(tan_list_temp > target_angle - pi/num_bin & tan_list_temp < target_angle + pi/num_bin, :);
                if isempty(target_points)
                    descriptor(j) = 0;
                else
                    if min(sqrt(target_points(:,1).^2 + target_points(:,2).^2)) > sensor_range
                        descriptor(j) = 0;
                    else
                        descriptor(j) = min(sqrt(target_points(:,1).^2 + target_points(:,2).^2));
                    end
                end
            end
            descriptor_list(i,:) = descriptor;
        end

        csvwrite(lidar_save_path, descriptor_list);

        pc_descriptor_rot_inv = zeros(length(descriptor_list), sensor_range/bin_size);
        for i = 1:length(descriptor_list)
            range_angle_descriptor = zeros(360, sensor_range/bin_size);

            pc_descriptor = descriptor_list(i,:);
            for j = 1:360
                if pc_descriptor(j) ~= 0
                    range_angle_descriptor(j,ceil(pc_descriptor(j)/bin_size)) = 1;
                end
            end
            rot_inv_descriptor = sum(range_angle_descriptor);

            pc_descriptor_rot_inv(i,:) = rot_inv_descriptor;
        end

        csvwrite(lidar_rotinv_save_path, pc_descriptor_rot_inv);


        disp(['KITTI sequence ', seq_name, ' LiDAR descriptor is generated.']);
    else
        disp('skip LiDAR descriptor.');
    end
end