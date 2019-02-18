clear; clc; close all;
addpath('/usr/local/gtsam_toolbox');
import gtsam.*

SHOW_ELLIPSES = false;

%% Read files
ak1_gt = csvread('ak1_gps_coords.csv');
ak2_gt = csvread('ak2_gps_coords.csv');
ak1_odom_raw = csvread('ak1_odom_poses.csv');
ak2_odom_raw = csvread('ak2_odom_poses.csv');
ak1_ak2_bearings = csvread('ak1_ak2_bearings.csv');
ak2_ak1_bearings = csvread('ak2_ak1_bearings.csv');

ODOM_SCALE = 0.92; %% What is this for ? 

%% Bearing offsets
ak1_bearing_offset = deg2rad(-8); %What is this for ? 
ak2_bearing_offset = deg2rad(170); %

%% Compute odometry tfs (per second)
start_time = floor(min(ak1_odom_raw(1, 1), ak2_odom_raw(1, 1)));
end_time = floor(max(ak1_odom_raw(end, 1), ak2_odom_raw(end, 1)));

% ak1
ak1_odom = containers.Map;
ak1_odom_aligned = [ak1_gt(1, 2), ak1_gt(1, 3), ak1_bearing_offset];
last_time = -1;
last_x = ak1_odom_raw(1, 2);
last_y = ak1_odom_raw(1, 3);
last_th = ak1_odom_raw(1, 4);
for i=2:size(ak1_odom_raw, 1)
    cur_time = floor(ak1_odom_raw(i, 1));
    if last_time == -1 || cur_time > last_time
        cur_x = ak1_odom_raw(i, 2);
        cur_y = ak1_odom_raw(i, 3);
        cur_th = ak1_odom_raw(i, 4);

        dx = (cur_x - last_x) * ODOM_SCALE;
        dy = (cur_y - last_y) * ODOM_SCALE;
        dthr = cur_th - last_th;

        % Convert dx, dy to robot frame
        dxr = dx * cos(-cur_th) - dy * sin(-cur_th);
        dyr = dx * sin(-cur_th) + dy * cos(-cur_th);

        ak1_odom(num2str(cur_time - start_time)) = [dxr, dyr, dthr];
        
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

% ak2
ak2_odom = containers.Map;
ak2_odom_aligned = [ak2_gt(1, 2), ak2_gt(1, 3), ak2_bearing_offset];
last_time = -1;
last_x = ak2_odom_raw(1, 2);
last_y = ak2_odom_raw(1, 3);
last_th = ak2_odom_raw(1, 4);
for i=2:size(ak2_odom_raw, 1)
    cur_time = floor(ak2_odom_raw(i, 1));
    if last_time == -1 || cur_time > last_time
        cur_x = ak2_odom_raw(i, 2);
        cur_y = ak2_odom_raw(i, 3);
        cur_th = ak2_odom_raw(i, 4);

        dx = (cur_x - last_x) * ODOM_SCALE;
        dy = (cur_y - last_y) * ODOM_SCALE;
        dthr = cur_th - last_th;

        % Convert dx, dy to robot frame
        dxr = dx * cos(-cur_th) - dy * sin(-cur_th);
        dyr = dx * sin(-cur_th) + dy * cos(-cur_th);

        ak2_odom(num2str(cur_time - start_time)) = [dxr, dyr, dthr];
        
        % Compute aligned odom
        theta = ak2_odom_aligned(end, 3) + dthr;
        x = ak2_odom_aligned(end, 1) + dxr * cos(theta) - dyr * sin(theta);
        y = ak2_odom_aligned(end, 2) + dxr * sin(theta) + dyr * cos(theta);
        ak2_odom_aligned = [ak2_odom_aligned; [x, y, theta]];
        
        last_time = cur_time;
        last_x = cur_x;
        last_y = cur_y;
        last_th = cur_th;
    end
end

%% Store bearings by time
ak1_ak2_bearings_map = containers.Map;
for i=1:size(ak1_ak2_bearings, 1)
    ak1_ak2_bearings_map(num2str(floor(ak1_ak2_bearings(i, 1) - start_time))) = deg2rad(ak1_ak2_bearings(i, 2));
end

ak2_ak1_bearings_map = containers.Map;
for i=1:size(ak2_ak1_bearings, 1)
    ak2_ak1_bearings_map(num2str(floor(ak2_ak1_bearings(i, 1) - start_time))) = deg2rad(ak2_ak1_bearings(i, 2));
end

%% Plot ground truth and odom
figure;
clf;
hold on;
axis([-100,-20,-10,70]);
set(gcf, 'Position', [0, 0, 500, 500]);
    
plot(ak1_gt(:, 2), ak1_gt(:, 3), 'r', 'LineWidth', 4);
plot(ak2_gt(:, 2), ak2_gt(:, 3), 'b', 'LineWidth', 4);
plot(ak1_odom_aligned(:, 1), ak1_odom_aligned(:, 2), 'r--', 'LineWidth', 2);
plot(ak2_odom_aligned(:, 1), ak2_odom_aligned(:, 2), 'b--', 'LineWidth', 2);

%% GTSAM
isamParams = ISAM2Params;
isamParams.setFactorization('QR');
isam = gtsam.ISAM2(isamParams);

newFactors = NonlinearFactorGraph;
newValues = Values;

% Covariances
odometryRSigma = 0.05;
odometryThetaSigma = 0.025;
measuredBearingNoiseSigma = 0.1;

priorNoise1 = noiseModel.Diagonal.Sigmas([1e-6; 1e-6; 1e-6]);
% priorNoise2 = noiseModel.Diagonal.Sigmas([10; 10; 5]);
priorNoise2 = priorNoise1;
odometryNoise = noiseModel.Diagonal.Sigmas([odometryRSigma; odometryRSigma; odometryThetaSigma]);
odometryNoNoise = noiseModel.Diagonal.Sigmas([0; 0; 0]);
bearingNoise = noiseModel.Diagonal.Sigmas(measuredBearingNoiseSigma);

%% Construct and solve graph
ak1_bearing_measurements = []; % for plotting
ak2_bearing_measurements = []; % for plotting
ak1_init = 0;
ak2_init = 0;
for i=0:end_time - start_time
    i
    key = num2str(i);
    
    % ak1 odom
    if ak1_odom.isKey(key)
        if ~ak1_init
            ak1_init = 1;

            ak1_cur_pose = Pose2(ak1_gt(1, 2), ak1_gt(1, 3), ak1_bearing_offset);
            newFactors.add(PriorFactorPose2(symbol('a', i), ak1_cur_pose, priorNoise1));
            newValues.insert(symbol('a', i), ak1_cur_pose);
        else
            odom = ak1_odom(key);
            ak1_change = Pose2(odom(1), odom(2), odom(3));
            ak1_cur_pose = ak1_cur_pose.compose(ak1_change);
            if odom(1) == 0 && odom(2) == 0
                noise = odometryNoNoise;
            else
                noise = odometryNoise;
            end
            newFactors.add(BetweenFactorPose2(last_ak1_symbol, symbol('a', i), ak1_change, noise));
            newValues.insert(symbol('a', i), ak1_cur_pose);
        end
        last_ak1_symbol = symbol('a', i);
    end
    
    % ak2 odom
    if ak2_odom.isKey(key)
        if ~ak2_init
            ak2_init = 1;

            ak2_cur_pose = Pose2(ak2_gt(1, 2), ak2_gt(1, 3), ak2_bearing_offset);
            newFactors.add(PriorFactorPose2(symbol('b', i), ak2_cur_pose, priorNoise2));
            newValues.insert(symbol('b', i), ak2_cur_pose);
        else
            odom = ak2_odom(key);
            ak2_change = Pose2(odom(1), odom(2), odom(3));
            ak2_cur_pose = ak2_cur_pose.compose(ak2_change);
            if odom(1) == 0 && odom(2) == 0
                noise = odometryNoNoise;
            else
                noise = odometryNoise;
            end
            newFactors.add(BetweenFactorPose2(last_ak2_symbol, symbol('b', i), ak2_change, noise));
            newValues.insert(symbol('b', i), ak2_cur_pose);
        end
        last_ak2_symbol = symbol('b', i);
    end

    % ak1->ak2 bearings
    if ak1_ak2_bearings_map.isKey(key) && ak1_odom.isKey(key) && ak2_odom.isKey(key)
        ak1_bearing_measurements = [ak1_bearing_measurements, i];
        bearing = ak1_ak2_bearings_map(key);
        newFactors.add(BearingFactorPose2(symbol('a', i), symbol('b', i), Rot2(bearing), bearingNoise));
    end
    
    % ak2->ak1 bearings
    if ak2_ak1_bearings_map.isKey(key) && ak1_odom.isKey(key) && ak2_odom.isKey(key)
        ak2_bearing_measurements = [ak2_bearing_measurements, i];
        bearing = ak2_ak1_bearings_map(key);
        newFactors.add(BearingFactorPose2(symbol('b', i), symbol('a', i), Rot2(bearing), bearingNoise));
    end
end

LMParams = LevenbergMarquardtParams;
LMParams.setLinearSolverType('MULTIFRONTAL_QR');

optimizer = LevenbergMarquardtOptimizer(newFactors, newValues, LMParams);
result = optimizer.optimize();

hold on;
axis equal;
h = zeros(7, 1);

% Ground truth & Uncorrected
h(1) = plot(ak1_gt(:, 2), ak1_gt(:, 3), 'r', 'LineWidth', 3);
h(2) = plot(ak2_gt(:, 2), ak2_gt(:, 3), 'b', 'LineWidth', 3);
h(3) = plot(ak1_odom_aligned(:, 1), ak1_odom_aligned(:, 2), 'r--');
h(4) = plot(ak2_odom_aligned(:, 1), ak2_odom_aligned(:, 2), 'b--');

% ISAM Result
if SHOW_ELLIPSES
    marginals = Marginals(newFactors, result);
    plot2DTrajectory(result, 'g.', marginals);
else
    plot2DTrajectory(result, 'g.');
end

% Bearings
for t=1:size(ak1_bearing_measurements, 2)
    s = result.at(symbol('a', ak1_bearing_measurements(t)));
    e = result.at(symbol('b', ak1_bearing_measurements(t)));
    if s.x ~= e.x && s.y ~= e.y
        arrow3([s.x, s.y], [e.x, e.y], 'r:', 0.25, 0.5);
    end
end

for t=1:size(ak2_bearing_measurements, 2)
    s = result.at(symbol('b', ak2_bearing_measurements(t)));
    e = result.at(symbol('a', ak2_bearing_measurements(t)));
    if s.x ~= e.x && s.y ~= e.y
        arrow3([s.x, s.y], [e.x, e.y], 'b:', 0.25, 0.5);
    end
end

h(5) = plot(NaN,NaN,'r:');
h(6) = plot(NaN,NaN,'b:');
h(7) = plot(NaN,NaN,'g.');

legend(h, ...
    'Rover 1 Ground Truth',...
    'Rover 2 Ground Truth',...
    'Rover 1 Odometry',...
    'Rover 2 Odometry',...
    'Rover 1 -> Rover 2 Bearing Measurement',...
    'Rover 2 -> Rover 1 Bearing Measurement',...
    'Estimated Route',...
    'Location', 'southwest');
xlabel('Distance (meters)');
ylabel('Distance (meters)');
