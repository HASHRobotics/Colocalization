clear; clc; close all;
rng(0);
addpath('/usr/local/gtsam_toolbox');
import gtsam.*

h = zeros(4, 1);

%% Read files
OFFSET_MEASURED = 1547534811;
OFFSET_TRUE = 9381;

gt = csvread('ground_truth.csv');
gt = gt(9381:end, :);
odom = csvread('odom1.csv');
odom = odom(9381:end, :);
odom(:, 4) = deg2rad(odom(:, 4));
bearings = csvread('rs_detection_2_bearing.csv');
bearings(:, 2) = -bearings(:, 2) + pi/2;
ranges = csvread('range1.csv');
ranges = ranges(:, 2:3);

%% Compute Odom Tfs per second
start_time = odom(1, 1) - OFFSET_TRUE;
end_time = odom(end, 1) - OFFSET_TRUE;

ak1_odom = containers.Map;
ak1_odom_aligned = [gt(1, 2), gt(1, 3), 0];
last_time = -1;
last_x = odom(1, 2);
last_y = odom(1, 3);
last_th = odom(1, 4);
for i=2:size(odom, 1)
    cur_time = floor(odom(i, 1)) - OFFSET_TRUE;
    if last_time == -1 || cur_time > last_time
        cur_x = odom(i, 2);
        cur_y = odom(i, 3);
        cur_th = odom(i, 4);

        dx = (cur_x - last_x);
        dy = (cur_y - last_y);
        dthr = cur_th - last_th;

        % Convert dx, dy to robot frame
        dxr = dx * cos(-cur_th) - dy * sin(-cur_th);
        dyr = dx * sin(-cur_th) + dy * cos(-cur_th);
        
        % Add noise to r and th to simulate odometry
        r = sqrt(dxr^2 + dyr^2);
        dr = r + normrnd(0, 0.5 * r);
        dthr = dthr + normrnd(0, 0.025 * r);
        if r == 0
            scale = 1;
        else
            scale = dr / r;
        end
        dxr = dxr * scale;
        dyr = dyr * scale;

        ak1_odom(num2str(floor(cur_time - start_time))) = [dxr, dyr, dthr];
        
        % Compute aligned odom
        theta = ak1_odom_aligned(end, 3) + dthr;
        x = ak1_odom_aligned(end, 1) + dxr * cos(theta) - dyr * sin(theta);
        y = ak1_odom_aligned(end, 2) + dxr * sin(theta) + dyr * cos(theta);
        ak1_odom_aligned = [ak1_odom_aligned; [x, y, theta]];

        last_time = cur_time;
        last_x = cur_x;
        last_y = cur_y;
        last_th = cur_th;
    end
end

%% Store bearings by time
bearings_map = containers.Map;
for i=1:size(bearings, 1)
    if ~isnan(bearings(i,2))
        time = bearings(i, 1) / 1e9 - OFFSET_MEASURED;
        bearings_map(num2str(floor(time - start_time))) = bearings(i, 2);
    end
end

%% Store range by time
ranges_map = containers.Map;
for i=1:size(ranges, 1)
    time = ranges(i, 1) / 1e9 - OFFSET_MEASURED;
    ranges_map(num2str(floor(time - start_time))) = ranges(i, 2);
end

%% Plot ground truth and odom
figure;
clf;
hold on;
axis equal;
box on;

    
h(1) = plot(gt(:, 2), gt(:, 3), 'k', 'LineWidth', 2);
h(2) = plot(ak1_odom_aligned(:, 1), ak1_odom_aligned(:, 2), 'g--', 'LineWidth', 2);

%% Setup GTSAM
newFactors = NonlinearFactorGraph;
newValues = Values;

% Covariances
odometryRSigma = 0.5;
odometryThetaSigma = 0.025;
measuredBearingNoiseSigma = 0.01;
measuredRangeNoiseSigma = 0.1;

odometryNoNoise = noiseModel.Diagonal.Sigmas([0; 0; 0]);

%% Construct and solve graph
BEARING = true;
RANGE = true;

% Insert basestation
base_symbol = symbol('b', 0);
base_pose = Pose2(0, 0, 0);
newFactors.add(PriorFactorPose2(base_symbol, base_pose, noiseModel.Diagonal.Sigmas([1e-6; 1e-6; 1e-6])));
newValues.insert(base_symbol, base_pose);

bearing_measurements = []; % for plotting
range_measurements = []; % for plotting
ak1_init = 0;
for i=0:end_time - start_time
    i
    key = num2str(i);
    
    % ak1 odom
    if ak1_odom.isKey(key)
        if ~ak1_init
            ak1_init = 1;

            ak1_cur_pose = Pose2(gt(1, 2), gt(1, 3), 0);
            newFactors.add(PriorFactorPose2(symbol('a', i), ak1_cur_pose, noiseModel.Diagonal.Sigmas([1e-6; 1e-6; 1e-6])));
            newValues.insert(symbol('a', i), ak1_cur_pose);
        else
            odom = ak1_odom(key);
            ak1_change = Pose2(odom(1), odom(2), odom(3));
            ak1_cur_pose = ak1_cur_pose.compose(ak1_change);
            if odom(1) == 0 && odom(2) == 0
                noise = odometryNoNoise;
            else
                r = sqrt(ak1_change.x^2 + ak1_change.y^2);
                th = ak1_change.theta;
                noise = noiseModel.Diagonal.Sigmas([odometryRSigma * r; odometryRSigma * r; odometryThetaSigma * th]);
            end
            newFactors.add(BetweenFactorPose2(last_ak1_symbol, symbol('a', i), ak1_change, noise));
            newValues.insert(symbol('a', i), ak1_cur_pose);
        end
        last_ak1_symbol = symbol('a', i);

        % bearings
        if BEARING && ~RANGE
            if bearings_map.isKey(key)
                bearing_measurements = [bearing_measurements, i];
                bearing = bearings_map(key);
                newFactors.add(BearingFactorPose2(base_symbol, symbol('a', i), Rot2(bearing), noiseModel.Diagonal.Sigmas(measuredBearingNoiseSigma)));
            end
        end
        if RANGE && ~BEARING
            if ranges_map.isKey(key)
                range_measurements = [range_measurements, i];
                range = ranges_map(key);
                newFactors.add(RangeFactorPose2(base_symbol, symbol('a', i), range, noiseModel.Diagonal.Sigmas(measuredRangeNoiseSigma)));
            end
        end
        if RANGE && BEARING
            if bearings_map.isKey(key) && ranges_map.isKey(key)
                bearing_measurements = [bearing_measurements, i];
                range_measurements = [range_measurements, i];
                bearing = bearings_map(key);
                range = ranges_map(key);
                newFactors.add(BearingRangeFactorPose2(...
                    base_symbol,...
                    symbol('a', i),...
                    Rot2(bearing),...
                    range,...
                    noiseModel.Diagonal.Sigmas([measuredBearingNoiseSigma; measuredRangeNoiseSigma])));
            end
            if bearings_map.isKey(key) && ~ranges_map.isKey(key)
                bearing_measurements = [bearing_measurements, i];
                bearing = bearings_map(key);
                newFactors.add(BearingFactorPose2(base_symbol, symbol('a', i), Rot2(bearing), noiseModel.Diagonal.Sigmas(measuredBearingNoiseSigma)));
            end
            if ranges_map.isKey(key) && ~bearings_map.isKey(key)
                range_measurements = [range_measurements, i];
                range = ranges_map(key);
                newFactors.add(RangeFactorPose2(base_symbol, symbol('a', i), range, noiseModel.Diagonal.Sigmas(measuredRangeNoiseSigma)));
            end
        end
    end
end

LMParams = LevenbergMarquardtParams;
LMParams.setLinearSolverType('MULTIFRONTAL_QR');

optimizer = LevenbergMarquardtOptimizer(newFactors, newValues, LMParams);
result = optimizer.optimize();
plot2DTrajectory(result, 'r');
xlabel('Distance (meters)');
ylabel('Distance (meters)');
title({'Trajectory: 2','Camera: Pan-mounted', 'Method: Automatic','Corrected odometry using relative range and bearing measurements'}, 'FontSize',14)
h(3) = plot(NaN,NaN,'r');
h(4) = plot(NaN,NaN,'b-.');
k = keys(ranges_map); 
for t=1:size(ranges_map)
    key = k{t};
    s = result.at(base_symbol);
    try
        e = result.at(symbol('a', str2double(key)));
        if s.x ~= e.x && s.y ~= e.y && ~isKey(bearings_map,key)
            disp("It's a range only measurement")
            arrow3([s.x, s.y], [e.x, e.y], 'b-.', 0.25, 0.5);
        end
    catch
        warning("Couldn't find key", key);
    end
end

k = keys(bearings_map);
for t=1:size(bearings_map)
    key = k{t};
    s = result.at(base_symbol);
    try
        e = result.at(symbol('a', str2double(key)));
        if s.x ~= e.x && s.y ~= e.y && isKey(ranges_map, key)
            disp("It's a range and bearing measurement")
            arrow3([s.x, s.y], [e.x, e.y], 'b-.', 0.25, 0.5);
        end
    catch
        warning("Couldn't find key");
        warning(key);
    end
end
legend(h,' Ground truth path', ' Odometry', ' Estimated path', ' Range and bearing measurements', 'FontSize',12, 'Location','southwest')