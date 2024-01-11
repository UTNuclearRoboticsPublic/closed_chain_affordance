%--------------------------------------------------------------------------
% Author: Crasun Jans (Janak Panthi)
% Date: 12/2023
% Description: 
% This MATLAB script extracts joint states and end-effector position data
% from specified actual and predicted data files, generating two subplots
% that showcase the end-effector position and joint states over time. It
% also incorporates a postfix to the plot titles derived from the current 
% directory name, formatted to suit title conventions.
%
% Usage:
%   [Explain how to use the code, including any input/output parameters]
%
% Example:
%   [Provide an example of how to use the code, if applicable]
%
% Dependencies:
%   None
%
% References:
%   First usage in paper: Panthi, Affordance as a Closure to Open-Chain 
% Serial Manipulators and Numerical Inverse Kinematics for Closed-Chain 
% Mechanisms
%--------------------------------------------------------------------------
clear all;
close all;
warning('off');
clc;

%% Get title postfix for plots from current directory name
% Get the folder name where this script lives
script_dir = fileparts(mfilename('fullpath'));
[~, current_dir] = fileparts(script_dir);

% Extract parts from the directory name
parts = strsplit(current_dir, '_');

% List of lowercase words to exclude from capitalization
excludeList = {'and', 'or', 'but', 'for', 'nor', 'so', 'yet', 'a', 'an', 'the', 'at', 'by', 'for', 'in', 'of', 'on', 'to', 'up', 'as', 'it', 'is'};

% Capitalize each part, excluding specified words
title_postfix = '';
for i = 1:length(parts)
    word = lower(parts{i});
    if ~ismember(word, excludeList)
        word = [upper(word(1)), word(2:end)];
    end
    title_postfix = [title_postfix ' ' word];
end

% Remove leading space
title_postfix = strtrim(title_postfix);

% Remove any trailing numbers
if isstrprop(title_postfix(end), 'digit') 
    title_postfix =  title_postfix(1:end-1);
end

%% Read CSV files
pred_data_relative_filepath = fullfile(script_dir, 'pred_tf_and_joint_states_data.csv'); % can specify relative filepath if needed
act_data_relative_filepath = fullfile(script_dir, 'act_tf_and_joint_states_data.csv');
df_pred = readtable(pred_data_relative_filepath);
df_act = readtable(act_data_relative_filepath);

%% Create a figure with subplots
fig = figure; 

% Subplot for EE position
% ax_ee = subplot(1, 2, 1, 'Parent', fig, 'Projection', 'orthographic');
ax_ee = subplot(1, 1, 1, 'Parent', fig, 'Projection', 'orthographic');

plot3(ax_ee, ...
    df_pred{:, 'PredEEX'} * 1000, ...
    df_pred{:, 'PredEEY'} * 1000, ...
    df_pred{:, 'PredEEZ'} * 1000, ...
    'b-o', 'LineWidth', 5, 'DisplayName', 'Predicted');
hold(ax_ee, 'on');
plot3(ax_ee, ...
    df_act{:, 'ActEEX'} * 1000, ...
    df_act{:, 'ActEEY'} * 1000, ...
    df_act{:, 'ActEEZ'} * 1000, ...
    'k-o', 'LineWidth', 5, 'DisplayName', 'Actual');

% Subplot settings--
axis equal
grid on

% Subplot labels
xlabel(ax_ee, 'x (mm)');
ylabel(ax_ee, ['y (mm), ' ' [' sprintf('%.1f, %.1f', ylim) ']' ]);
zlabel(ax_ee, ['z (mm), ' ' [' sprintf('%.1f, %.1f', zlim) ']' ]);
legend(ax_ee, 'Location', 'best'); % Adjust legend location
title(ax_ee, ['EE Position - ', title_postfix]);

% Position legend manually out of the way
% legend_position = [0.33, 0.74, 0.1, 0.1];  % [x, y, width, height]
% legend(ax_ee, 'Location', 'layout', 'Position', legend_position);
legend(ax_ee);

% Ensure all ticks are labeled
xtick_locations = get(ax_ee, 'XTick');
ytick_locations = get(ax_ee, 'YTick');
ztick_locations = get(ax_ee, 'ZTick');
set(ax_ee, 'XTick', xtick_locations, 'XTickLabel', num2str(xtick_locations.'));
% set(ax_ee, 'YTick', ytick_locations, 'YTickLabel', num2str(ytick_locations.'));
set(ax_ee, 'ZTick', ztick_locations, 'ZTickLabel', num2str(ztick_locations.'));
set(ax_ee, 'YTickLabel', []);


% Set fontsizes for various plot parameters
title_fontsize = 45;
label_fontsize = 35;
legend_fontsize = 30;
tick_fontsize = 38;

ax_ee.XAxis.FontSize = tick_fontsize; 
ax_ee.XAxis.FontWeight = "bold";
% Fix pertaining only this dataset-----------------------------------------
ax_ee.YAxis.Label.Position = ax_ee.YAxis.Label.Position + [25, -20, 0];
% ax_ee.ZAxis.Label.Position = ax_ee.ZAxis.Label.Position + [-200, 0, 0];


ax_ee.YAxis.Label.Rotation = -35;
ax_ee.YTick = [-15 -11]; % fix specific to this dataset
ax_ee.YAxis.FontSize = tick_fontsize -15;
%--------------------------------------------------------------------------
ax_ee.ZAxis.FontSize = tick_fontsize;
ax_ee.XLabel.FontSize = label_fontsize;
ax_ee.YLabel.FontSize = label_fontsize;
ax_ee.YAxis.FontWeight = "bold";
ax_ee.ZLabel.FontSize = label_fontsize;
ax_ee.ZAxis.FontWeight = "bold";
ax_ee.Legend.FontSize = legend_fontsize;
ax_ee.Title.FontSize = title_fontsize;

% Subplot for joint states
% Location of the plot
% subplot_padding = 0.1;%padding between the two subplots
% ax_ee_pos = get(ax_ee, 'Position');
% ax_joint_states_pos = ax_ee_pos(1)+ax_ee_pos(3)+subplot_padding; % x+width+padding
% ax_joint_states = subplot(1, 2, 2, 'Parent', fig, 'Position', [ax_joint_states_pos, ax_ee_pos(2), ax_ee_pos(3), ax_ee_pos(4)]); % y, width, height the same as ax_ee
fig2 = figure(2); 
ax_joint_states = subplot(1, 1, 1, 'Parent', fig2);


% % Timestamp resolution is in seconds, i.e. decimal parts are missing. So,
% % we fill out the missing parts evenly while preserving the overall time taken.
% % If all actual timestamps are the same then, we'll use the predicted timestamps
% % to construct this vector. This has happened with some data set due to a
% % certain ROS error.
timestamps = df_act{:, 'Timestamp'};
timestamps = timestamps - timestamps(1);
if ~all(timestamps==0)
    manual_timestamps = linspace(0, max(timestamps), length(timestamps));
else
    loop_rate = 6; %hz from data collection code and verified approx with video
    sampling_time = 1/loop_rate;
    manual_timestamps = 0:sampling_time:(length(timestamps)-1)*sampling_time;
    disp("All timestamps were the same. Now corrected based on real video and code sampling rate.")
    % manual_timestamps = linspace(0, max(df_pred{:,'Timestamp'}), length(timestamps));
end

% Joint state column headers
js_end_index = 61; % specify end to truncate saturated readings
joint_names = {'arm0_shoulder_yaw', 'arm0_shoulder_pitch', 'arm0_elbow_pitch', 'arm0_elbow_roll', 'arm0_wrist_pitch', 'arm0_wrist_roll'};

% Extract data and plot
for i = 1:length(joint_names)
    joint = joint_names{i};

    plot(ax_joint_states, manual_timestamps(1:js_end_index), df_act{:,joint}(1:js_end_index), 'LineWidth', 8, 'DisplayName', joint);
    hold(ax_joint_states, 'on');
end

% Subplot settings--
grid on


% Set fontsizes for various plot parameters
title_fontsize = 50;
label_fontsize = 50;
legend_fontsize = 40;
tick_fontsize = 50;
grid_lw = 1.5;

% Subplot labels
xlabel(ax_joint_states, 'time (s)');
ylabel(ax_joint_states, 'joint states (rad)');
legend(ax_joint_states,'Interpreter', 'none', 'Color', 'none'); % Don't interpret underscores as cues for subscripts, and don't fill the legend box
title(ax_joint_states, ['Actual Joint States vs. Time - ',title_postfix]);

% Set fontsizes for various plot parameters
ax_joint_states.XAxis.FontSize = tick_fontsize; 
ax_joint_states.XAxis.FontWeight = "bold";
ax_joint_states.YAxis.FontSize = tick_fontsize;
ax_joint_states.YAxis.FontWeight = "bold";
ax_joint_states.XLabel.FontSize = label_fontsize;
ax_joint_states.YLabel.FontSize = label_fontsize;
ax_joint_states.ZLabel.FontSize = label_fontsize;
ax_joint_states.Legend.FontSize = legend_fontsize;
ax_joint_states.Title.FontSize = title_fontsize;
ax_joint_states.GridLineWidth = grid_lw;
% ax_joint_states.Legend.Location = "east";

% Set legend position
legend_position = ax_joint_states.Legend.Position;
shift_amount_x = 0.4; 
shift_amount_y = 0.235; 
ax_joint_states.Legend.Position = [legend_position(1) + shift_amount_x, legend_position(2)+ shift_amount_y, legend_position(3), legend_position(4)];


% % Plot Error between predicted and actual EE cartesian trajectory
% fig3 = figure(3); 
% ax_ee_traj_error = subplot(1, 1, 1, 'Parent', fig3);
% 
% 
% L2 = [df_pred{:, 'PredEEX'} * 1000, ...
%     df_pred{:, 'PredEEY'} * 1000, ...
%     df_pred{:, 'PredEEZ'} * 1000];
% L1 = [df_act{:, 'ActEEX'} * 1000, ...
%     df_act{:, 'ActEEY'} * 1000, ...
%     df_act{:, 'ActEEZ'} * 1000];
% d = pdist2(L1, L2, 'euclidean', 'smallest', 1)
% max_d = max(d)
% min_d = min(d)
% length_L2 = length(L2)
% length_L1 = length(L1)
% length_d = length(d)
% 
% %Fit
% f = polyfit(df_pred{:, 'Timestamp'}, d,1) % linear fit
% length_f = length(f)
% length_timestamp = length(df_pred{:, 'Timestamp'})
% % Plot
% plot(ax_ee_traj_error, df_pred{:, 'Timestamp'}, d, 'b-o','LineWidth', 8);
% hold on
% plot(ax_ee_traj_error, df_pred{:, 'Timestamp'}, polyval(f, df_pred{:, 'Timestamp'}), 'r','LineWidth', 8);
% % Subplot settings--
% grid on
% 
% % Subplot labels
% xlabel(ax_ee_traj_error, 'predicted trajectory timestamp (s)');
% ylabel(ax_ee_traj_error, 'ee trajectory error (mm)');
% title(ax_ee_traj_error, ['EE Cartesian Trajectory Error vs. Time - ',title_postfix]);
% 
% % Set fontsizes for various plot parameters
% ax_ee_traj_error.XAxis.FontSize = tick_fontsize; 
% ax_ee_traj_error.XAxis.FontWeight = "bold";
% ax_ee_traj_error.YAxis.FontSize = tick_fontsize;
% ax_ee_traj_error.YAxis.FontWeight = "bold";
% ax_ee_traj_error.XLabel.FontSize = label_fontsize;
% ax_ee_traj_error.YLabel.FontSize = label_fontsize;
% ax_ee_traj_error.ZLabel.FontSize = label_fontsize;
% ax_ee_traj_error.Title.FontSize = title_fontsize;
% ax_ee_traj_error.GridLineWidth = grid_lw;

% Plot Error between predicted and actual EE cartesian trajectory
fig3 = figure(3); 
ax_ee_traj_error = subplot(1, 1, 1, 'Parent', fig3);


L2 = [df_pred{:, 'PredEEX'} * 1000, ...
    df_pred{:, 'PredEEY'} * 1000, ...
    df_pred{:, 'PredEEZ'} * 1000];
L1 = [df_act{:, 'ActEEX'} * 1000, ...
    df_act{:, 'ActEEY'} * 1000, ...
    df_act{:, 'ActEEZ'} * 1000];
% d = pdist2(L1, L2, 'euclidean', 'smallest', 1)
% Loop through each point in L2 and find the distance to the closest point in L1
distances = zeros(size(L2, 1), 1);

for i = 1:size(L2, 1)
    point_to_find = L2(i, :);  % Current point in L2

    % Calculate Euclidean distances between point_to_find and all points in L1
    distances_to_L1 = sqrt(sum((L1 - point_to_find).^2, 2));

    % Find the index of the two closest points in L1
    [~, min_indices] = mink(distances_to_L1, 2);

    % Extract the coordinates of the two closest points in L1
    closest_points = L1(min_indices, :);

    % Interpolate between the two closest points in L1
    t = linspace(0, 1, 100);  % Adjust the number of points as needed
    interpolated_points = closest_points(1, :) + t' * (closest_points(2, :) - closest_points(1, :));

    % Calculate Euclidean distances between point_to_find and all interpolated points
    distances_to_interpolated = sqrt(sum((interpolated_points - point_to_find).^2, 2));

    % Find the smallest distance to the interpolated points
    distances(i) = min(distances_to_interpolated);
end
d = distances
max_d = max(d)
min_d = min(d)
length_L2 = length(L2)
length_L1 = length(L1)
length_d = length(d)
ts = df_pred{:, 'Timestamp'};
%Fit
f = polyfit(ts, d,1) % linear fit
length_f = length(f)
length_timestamp = length(ts)
% Plot
plot(ax_ee_traj_error, ts, d, 'b-o','LineWidth', 8);
hold on
% plot(ax_ee_traj_error, ts, polyval(f, ts), 'r','LineWidth', 8);
% Subplot settings--
grid on

% Subplot labels
xlabel(ax_ee_traj_error, 'predicted trajectory timestamp (s)');
ylabel(ax_ee_traj_error, 'ee trajectory error (mm)');
title(ax_ee_traj_error, ['EE Cartesian Trajectory Error vs. Time - ',title_postfix]);

% Set fontsizes for various plot parameters
ax_ee_traj_error.XAxis.FontSize = tick_fontsize; 
ax_ee_traj_error.XAxis.FontWeight = "bold";
ax_ee_traj_error.YAxis.FontSize = tick_fontsize;
ax_ee_traj_error.YAxis.FontWeight = "bold";
ax_ee_traj_error.XLabel.FontSize = label_fontsize;
ax_ee_traj_error.YLabel.FontSize = label_fontsize;
ax_ee_traj_error.ZLabel.FontSize = label_fontsize;
ax_ee_traj_error.Title.FontSize = title_fontsize;
ax_ee_traj_error.GridLineWidth = grid_lw;


