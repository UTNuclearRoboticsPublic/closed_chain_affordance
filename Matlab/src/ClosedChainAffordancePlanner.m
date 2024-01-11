%--------------------------------------------------------------------------
% Author: Crasun Jans (Janak Panthi)
% Date: 07/2023
% Description: 
%   This MATLAB script simulates execution of different affordance types by
%   a UR5 robot based on a novel closed-chain inverse kinematics approach.
%
% Usage:
%   [Explain how to use the code, including any input/output parameters]
%
% Example:
%   [Provide an example of how to use the code, if applicable]
%
% Dependencies:
%   Robotics System Toolbox
%
% References:
%   Panthi, Affordance as a Closure to Open-Chain Serial Manipulators and 
% Numerical Inverse Kinematics for Closed-Chain Mechanisms
%
%--------------------------------------------------------------------------

%% Clear variables and figures
close all
clear all
clf
clc

% Robot and Affordance Type
robotType = 'UR5';
affType = 'pure_trans';

% Algorithm control parameters
affStep = 0.05;
accuracy = 1*(1/100); % accuracy for error threshold
taskErrThreshold = accuracy*affStep;
closureErrThreshold = 1e-4;
maxItr = 50; % for IK solver
stepperMaxItr = 1; % for total steps
dt = 1e-2; % time step to compute joint velocities
delta_theta = 0.1;
pathComputerFlag = true;
taskOffset = 1;

% Build the robot and plot FK to validate configuration
[mlist, slist, thetalist0, Tsd, x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler,  azimuth, elevation] = RobotBuilder(robotType, affType);
figure(1)
robotType = 'UR3';
if strcmpi(robotType,'UR5')
    robot = loadrobot("universalUR5","DataFormat","column");
else
    robot = [];
end
screwPathMatrix = zeros(3,3,3);
[plotrepf, plotrepl, plotrepj, plotrept, plotrepn] = FKPlotter(mlist,slist,thetalist0, x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler,  azimuth, elevation, robotType, robot, screwPathMatrix); 

% Specify screw axis for pure translation
if strcmpi(affType,'pure_trans')
    slist(:,end) = [0 0 0 1 0 0]';
end

% Extract error frame
mErr = mlist(:,:,end);

% Loop iterators
stepperItr = 1; % increment iterator for the Stepper loop
stepperItrSuc = 1; % increment iterator for the Stepper loop for successful steps
ikIterHolder = []; % Holds IK iterations for successful steps

% Guesses
qp_guess = thetalist0(1:end-taskOffset);
qsb_guess = thetalist0(end-taskOffset+1:end);

% Stepping loop
while stepperItr<=stepperMaxItr

tic; % For computation time calculation

% Define Network Matrices as relevant Jacobian columns
rJ = JacobianSpace(slist,thetalist0); % robot Jacobian
Np = rJ(:,1:end-taskOffset); % primary - actuated
Ns = rJ(:,end-taskOffset+1:end); % secondary - unactuated


% Set desired secondary task (affordance and maybe gripper orientation) as 
% just a few radians away from the current position
qsd = thetalist0(end-taskOffset+1:end);
qsd(taskOffset) = qsd(taskOffset)+stepperItr*affStep;


% Set starting guess for the primary joint angles and compute forward
% kinematics
oldqp= zeros(size(Np,2),1); % for joint velocity calculation
qp = qp_guess;
qsb = qsb_guess;

% Set closure error to zero to enter the loop
errTwist = zeros(6,1);

% Check Newton-Raphson error
err = norm(qsd-qsb)>taskErrThreshold|| norm(errTwist)>closureErrThreshold;

% Implement Algorithm
ikIter = 1; % IK loop iterator
warning('off'); % turn MATLAB warnings off

errPlotMatrix = []; % Plotting matrix

while err && ikIter<maxItr

    %Update Jacobians
    thetalist =[qp; qsb]; 
    rJ = JacobianSpace(slist,thetalist);
    Np = rJ(:,1:end-taskOffset); % primary - actuated
    Ns = rJ(:,end-taskOffset+1:end); % secondary - unactuated

    % Compute primary joint angles
    qp_dot = (qp - oldqp)/dt;
    cJ = -pinv(Ns)*(Np+errTwist*pinv(qp_dot)); % Constraint Jacobian
    oldqp = qp;
    qp = qp + pinv(cJ)*(qsd-qsb);


    % Damped Least Squares
    % lambda = 1.2;
    % qp = qp +cJ'*pinv(cJ*cJ'+lambda^2*eye(size(cJ*cJ',1)))*(qsd-qsb);

    % Optimize closure error
    [qp, qsb, errTwist] = ClosureErrorOptimizer(mErr, slist, qp, qsb, Np, Ns, Tsd, taskOffset);


    % Store errors for plotting
    errPlotMatrix(ikIter,1) = ikIter;
    errPlotMatrix(ikIter,2) = norm(qsd-qsb);
    errPlotMatrix(ikIter,3) = norm(errTwist);
    
    % Check error as loop condition
err = norm(qsd-qsb)>taskErrThreshold|| norm(errTwist)>closureErrThreshold;

    % Increment loop iterator
    ikIter = ikIter+1;

end
warning('on');
elapsed_time = toc; % stop time computation
disp(['Elapsed Time: ' num2str(elapsed_time) ' seconds']);

% Determine success
success = ~err; % If no error and thetalist does not have NaNs
thetalist =[qp; qsb]; % store the very last values as thetalist

%Plot errors
figure(2)
subplot(2,1,1)
%Font sizes and linewidths
grid_fontsize = 50;
label_fontsize = 50;
title_fontsize = 50;
grid_lw = 1.5;
plot_lw = 8; %linewidth
plot(errPlotMatrix(:,1),errPlotMatrix(:,2), 'b', 'LineWidth', plot_lw);
set(gca, 'YScale', 'log');
set(gca, 'FontSize', grid_fontsize,  'FontWeight', 'bold');
grid on
set(gca, 'GridLineWidth', grid_lw);
title("Affordance Step Goal Error vs. Iteration for " + num2str(stepperItr) + "st Step", 'FontSize', title_fontsize);
xlabel("iterations", 'FontSize', label_fontsize,  'FontWeight', 'bold');
% xlim([1 9]);    % Pure rotation first step of 0.5rad
% xlim([1 8]);    % Pure rotation first step of 0.1rad
xlim([1 30]);    % Pure translation first step of 0.05m
% xlim([1 30]);    % Pure translation first step of 0.01m

% ee_error_plot_xlim = xlim(gca)
if strcmpi(affType,'pure_trans')
    yyaxis left
    ylabel("ee error, m", 'FontSize', label_fontsize,  'FontWeight', 'bold');
    % Pure translation first step of 0.05m
    ylim([10e-5 10e-2]);
    yticks([10^-3,10^-1]);
    % Pure translation first step of 0.01m
    % ylim([10e-6-10e-7 10e-3]);
    % yticks([10^-5, 10^-3]);
    yyaxis right
    set(gca, 'YColor', 'k');
    set(gca, 'YScale', 'log');
    % Pure translation first step of 0.05m
    ylim([10e-5 10e-2]);
    yticks([10^-4, 10^-2]);
    % Pure translation first step of 0.01m
    % ylim([10e-6-10e-7 10e-3]);
    % yticks([10^-4, 10^-2]);
else
    yyaxis left
    ylabel("ee error, rad", 'FontSize', label_fontsize,  'FontWeight', 'bold');
    % Pure rotation first step of 0.5rad
    ylim([10e-4 10e-1]);
    yticks([10^-3,10^-1]);
    % Pure rotation first step of 0.1rad
    % ylim([10e-5 10e-2]);
    % yticks([10^-3,10^-1]);
    yyaxis right
    set(gca, 'YColor', 'k');
    set(gca, 'YScale', 'log');
    % Pure rotation first step of 0.5rad
    ylim([10e-4 10e-1]);
    yticks([10^-2,10^0]);
    % Pure rotation first step of 0.1rad
    % ylim([10e-5 10e-2]);
    % yticks([10^-4,10^-2]);
end

subplot(2,1,2)
plot(errPlotMatrix(:,1),errPlotMatrix(:,3), 'b', 'LineWidth', plot_lw);
set(gca, 'YScale', 'log');
set(gca, 'FontSize', grid_fontsize,  'FontWeight', 'bold');
grid on
set(gca, 'GridLineWidth', grid_lw);
title("Closure Error vs. Iteration for " + num2str(stepperItr) + "st Step", 'FontSize', title_fontsize);
xlabel("iterations", 'FontSize', label_fontsize,  'FontWeight', 'bold');
ylabel("closure error", 'FontSize', label_fontsize,  'FontWeight', 'bold');
% xlim([1 9]);    % Pure rotation first step of 0.5rad
% xlim([1 8]);    % Pure rotation first step of 0.1rad
xlim([1 30]);    % Pure translation first step of 0.05m
% xlim([1 30]);    % Pure translation first step of 0.01m


yyaxis left
% Pure translation first step of 0.05m
ylim([10e-7 10e-2]);
yticks([10^-5, 10^-3, 10^-1]);
% Pure translation first step of 0.01m
% ylim([10e-9 10e-4]);
% yticks([10^-7, 10^-5, 10^-3]);
% Pure rotation first step of 0.5rad
% ylim([10e-8 10e-2]);
% yticks([10^-7, 10^-5, 10^-3, 10^-1]);
% Pure rotation first step of 0.1rad
% ylim([10e-8 10e-3]);
% yticks([10^-7, 10^-5, 10^-3]);
yyaxis right
set(gca, 'YColor', 'k');
set(gca, 'YScale', 'log');
% Pure translation first step of 0.05m
ylim([10e-7 10e-2]);
yticks([10^-6, 10^-4, 10^-2]);
% Pure translation first step of 0.01m
% ylim([10e-9 10e-4]);
% yticks([10^-8, 10^-6, 10^-4]);
% Pure rotation first step of 0.5rad
% ylim([10e-8 10e-2]);
% yticks([10^-6, 10^-4, 10^-2]);
% Pure rotation first step of 0.1rad
% ylim([10e-8 10e-3]);
% yticks([10^-6, 10^-4, 10^-2]);

if success
    disp("Working m:")
    disp(stepperItr)
    ikIterHolder = [ikIterHolder ikIter-1];
    if pathComputerFlag
    % Compute the screw path to plot as well
    screwPathStart = FKinSpace(mlist(:,:,end-2), slist(:,1:end-2), thetalist(1:end-2));% Starting guess for all relevant frames/tasks
    if strcmpi(affType,'pure_trans')
        iterations = 5;
        start_offset = [0.1; 0; 0];
        screwPathStart(1:3, 4) = screwPathStart(1:3, 4) + start_offset; % offset the path such that the beginning of the path coincides with the start config of the robot
        screwPath  = screwPathCreator(rJ(:,end), screwPathStart, -delta_theta, iterations);
    else
        iterations = 64;
        screwPath  = screwPathCreator(rJ(:,end), screwPathStart, delta_theta, iterations);
    end
    screwPathMatrix = reshape(screwPath(1:3, 4, :), 3, [])'; % get xyz coordinates and put them in an N x 3 form
    pathComputerFlag = false; % to compute only once

    end
    animPlotMatrix(:,stepperItrSuc) = thetalist;

    %Update the guess for next iteration
    qp_guess = thetalist(1:end-taskOffset);
    qsb_guess = thetalist(end-taskOffset+1:end);
    stepperItrSuc = stepperItrSuc+1;
end

% Increment stepper loop iterator
stepperItr = stepperItr +1;
end

% Plot animation
figure(1)
disp("minimum IK iteration:");
min(ikIterHolder)
disp("maximum IK iteration:");
max(ikIterHolder)

% Delete the manipulator initial config plot references
% delete(plotrepf);
% delete(plotrepl);
% delete(plotrepj);
% delete(plotrept);
if strcmpi(robotType, 'UR5')
    delete(plotrepn);   
end

% Animate stored configurations
for ikIter = 1:1:stepperItrSuc-1
    [plotrepf, plotrepl, plotrepj, plotrept, plotrepn] = FKPlotter(mlist,slist,animPlotMatrix(:,ikIter), x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler,  azimuth, elevation, robotType, robot, screwPathMatrix); 
    drawnow;
    pause(0.001);
    if ikIter~=stepperItrSuc-1
    % delete(plotrepf);
    % delete(plotrepl);
    % delete(plotrepj);
    % delete(plotrept);
        if strcmpi(robotType, 'UR5')
            delete(plotrepn);   
        end
    end
end