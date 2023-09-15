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
affType = 'pure_rot';

% Algorithm control parameters
affStep = 0.01;
accuracy = 1*(1/100); % accuracy for error threshold
taskErrThreshold = accuracy*affStep;
maxItr = 50; % for IK solver
stepperMaxItr = 75; % for total steps
dt = 1e-2; % time step to compute joint velocities
delta_theta = 0.1;
pathComputerFlag = true;
taskOffset = 1;

% Build the robot and plot FK to validate configuration
[mlist, slist, thetalist0, Tsd, x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler,  azimuth, elevation] = RobotBuilder(robotType, affType);
figure(3)
if strcmpi(robotType,'UR5')
    robot = loadrobot("universalUR5","DataFormat","column");
else
    robot = [];
end
screwPathMatrix = zeros(3,3,3);
FKPlotter(mlist,slist,thetalist0, x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler,  azimuth, elevation, robotType, robot, screwPathMatrix); 

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
err = norm(qsd-qsb)>taskErrThreshold;

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
    err = norm(qsd-qsb)>taskErrThreshold;

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
figure(10)
subplot(2,1,1)
plot(errPlotMatrix(:,1),errPlotMatrix(:,2));
set(gca, 'YScale', 'log');
title("EE Error vs. Iteration for " + num2str(stepperItr) + "st Step");
xlabel("iterations");
ylabel("ee error");
subplot(2,1,2)
plot(errPlotMatrix(:,1),errPlotMatrix(:,3));
set(gca, 'YScale', 'log');
title("Closure Error vs. Iteration for " + num2str(stepperItr) + "st Step");
xlabel("iterations");
ylabel("closure error");

if success
    disp("Working m:")
    disp(stepperItr)
    ikIterHolder = [ikIterHolder ikIter-1];
    if pathComputerFlag
    % Compute the screw path to plot as well
    iterations = 64;
    screwPathStart = FKinSpace(mlist(:,:,end-2), slist(:,1:end-2), thetalist(1:end-2));% Starting guess for all relevant frames/tasks
    screwPath  = screwPathCreator(rJ(:,end), screwPathStart, delta_theta, iterations);
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
figure(3)
disp("minimum IK iteration:");
min(ikIterHolder)
disp("maximum IK iteration:");
max(ikIterHolder)
for ikIter = 1:1:stepperItrSuc-1
    [plotrepf, plotrepl, plotrepj, plotrept, plotrepn] = FKPlotter(mlist,slist,animPlotMatrix(:,ikIter), x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler,  azimuth, elevation, robotType, robot, screwPathMatrix); 
    drawnow;
    pause(0.1);
    if ikIter~=stepperItrSuc-1
    delete(plotrepf);
    delete(plotrepl);
    delete(plotrepj);
    delete(plotrept);
        if strcmpi(robotType, 'UR5')
            delete(plotrepn);   
        end
    end
end