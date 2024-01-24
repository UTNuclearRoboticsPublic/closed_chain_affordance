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
ax_ee = subplot(1, 2, 1, 'Parent', fig, 'Projection', 'orthographic');
plot3(ax_ee, ...
    df_pred{:, 'PredEEX'} * 1000, ...
    df_pred{:, 'PredEEY'} * 1000, ...
    df_pred{:, 'PredEEZ'} * 1000, ...
    'b-o', 'LineWidth', 3, 'DisplayName', 'Predicted');
hold(ax_ee, 'on');
plot3(ax_ee, ...
    df_act{:, 'ActEEX'} * 1000, ...
    df_act{:, 'ActEEY'} * 1000, ...
    df_act{:, 'ActEEZ'} * 1000, ...
    'k-o', 'LineWidth', 3, 'DisplayName', 'Actual');

% Subplot settings--
axis equal
grid on

% Subplot labels
xlabel(ax_ee, 'x (mm)');
ylabel(ax_ee, 'y (mm)');
zlabel(ax_ee, 'z (mm)');
legend(ax_ee, 'Location', 'best'); % Adjust legend location
title(ax_ee, ['EE Position - ', title_postfix]);

% Position legend manually out of the way
legend_position = [0.33, 0.74, 0.1, 0.1];  % [x, y, width, height]
legend(ax_ee, 'Location', 'layout', 'Position', legend_position);

% Ensure all ticks are labeled
xtick_locations = get(ax_ee, 'XTick');
set(ax_ee, 'XTick', xtick_locations, 'XTickLabel', num2str(xtick_locations.'));

% Set fontsizes for various plot parameters
title_fontsize = 25;
label_fontsize = 20;
legend_fontsize = 20;
tick_fontsize = 18;

ax_ee.XAxis.FontSize = tick_fontsize-7; 
ax_ee.YAxis.FontSize = tick_fontsize;
ax_ee.ZAxis.FontSize = tick_fontsize;
ax_ee.XLabel.FontSize = label_fontsize;
ax_ee.YLabel.FontSize = label_fontsize;
ax_ee.ZLabel.FontSize = label_fontsize;
ax_ee.Legend.FontSize = legend_fontsize;
ax_ee.Title.FontSize = title_fontsize;

% Subplot for joint states
ax_joint_states = subplot(1, 2, 2, 'Parent', fig, 'Position', [0.45, 0.1, 0.4, 0.8]);

% Timestamp resolution is in seconds, i.e. decimal parts are missing. So,
% we fill out the missing parts evenly while preserving the overall time taken.
timestamps = df_act{:, 'Timestamp'};
timestamps = timestamps - timestamps(1);
manual_timestamps = linspace(0, max(timestamps), length(timestamps));

% Joint state column headers
js_end_index = 50; % specify end to truncate saturated readings
joint_names = {'arm0_shoulder_yaw', 'arm0_shoulder_pitch', 'arm0_elbow_pitch', 'arm0_elbow_roll', 'arm0_wrist_pitch', 'arm0_wrist_roll'};

% Extract data and plot
for i = 1:length(joint_names)
    joint = joint_names{i};

    plot(ax_joint_states, manual_timestamps(1:js_end_index), df_act{:,joint}(1:js_end_index), 'LineWidth', 2, 'DisplayName', joint);
    hold(ax_joint_states, 'on');
end

% Subplot settings--
grid off

% Subplot labels
xlabel(ax_joint_states, 'time (s)');
ylabel(ax_joint_states, 'joint states (rad)');
legend(ax_joint_states,'Interpreter', 'none', 'Color', 'none'); % Don't interpret underscores as cues for subscripts, and don't fill the legend box
title(ax_joint_states, ['Actual Joint States vs. Time - ',title_postfix]);

% Set fontsizes for various plot parameters
ax_joint_states.XAxis.FontSize = tick_fontsize; 
ax_joint_states.YAxis.FontSize = tick_fontsize;
ax_joint_states.XLabel.FontSize = label_fontsize;
ax_joint_states.YLabel.FontSize = label_fontsize;
ax_joint_states.ZLabel.FontSize = label_fontsize;
ax_joint_states.Legend.FontSize = legend_fontsize;
ax_joint_states.Title.FontSize = title_fontsize;