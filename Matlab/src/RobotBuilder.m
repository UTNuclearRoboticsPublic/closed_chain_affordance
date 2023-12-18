function [mlist, slist, thetalist0, Tsd, x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler, azimuth, elevation] = RobotBuilder(robotType, affType)

switch robotType
    case '4-bar'
        %% 4-bar
        % Solve symbolically 
        % Define four bar parameters
        syms theta2s theta3s real
        L(1) = 1;
        L(2) = 1;
        L(3) = 1;
        L(4) = 1.4;
        theta1s = 0.5; %s is for absolute angles
        theta4s =0;
    
        % Vector Loop Equation
        eqn = L(1)*exp(-1i*theta1s)+L(2)*exp(-1i*(theta2s))-L(3)*exp(-1i*(theta3s))-L(4)*exp(-1i*(theta4s))==0;
    
        % Solve the equation
        solution = solve(eqn,[theta2s,theta3s]);
        theta2s = real(double(solution.theta2s(1)));
        theta3s = real(double(solution.theta3s(1)));
        thetalist0 = [theta1s -(theta1s-theta2s) -(pi-theta3s+theta2s) -theta3s]'; % these are relative angles
        
    
        % Set frames
        spaceFrame = eye(4);
        mlist(:,:,1) = spaceFrame;
        mlist(:,:,2) = tM('z',0,[L(1) 0 0]');
        mlist(:,:,3) = tM('z',0,[L(1)+L(2) 0 0]');
        mlist(:,:,4) = tM('z',0,[L(1)+L(2)+L(3) 0 0]');
        mlist(:,:,5) = tM('z',0,[L(1)+L(2)+L(3)+L(4) 0 0]');
    
        % Type and alignment of screw axes
        q(:,1) = mlist(1:3,4,1);
        q(:,2) = mlist(1:3,4,2);
        q(:,3) = mlist(1:3,4,3);
        q(:,4) = mlist(1:3,4,4);
    
        w(:,1) = [0 0 1]';
        w(:,2) = [0 0 1]'; 
        w(:,3) = [0 0 1]'; 
        w(:,4) = [0 0 1]'; 
        
    
        v(:,1) = -cross(w(:,1),q(:,1));
        v(:,2) = -cross(w(:,2),q(:,2));
        v(:,3) = -cross(w(:,3),q(:,3));
        v(:,4) = -cross(w(:,4),q(:,4));
    
        slist(:,1) = [w(:,1); v(:,1)];
        slist(:,2) = [w(:,2); v(:,2)];
        slist(:,3) = [w(:,3); v(:,3)];
        slist(:,4) = [w(:,4); v(:,4)];

        % % Affordance type
        % if strcmpi(affType,'pure_trans')
        %     slist(:,end) = [0 1 0 0 0 0]';
        % end

        % Closure error
        Tsd = tM('z',-pi,[0 0 0]'); % Desired error frame, ideally on the first joint but rotated by -180deg

        % Task indices
        x1Tindex = 4;
        x2Tindex = 3;

        % Plotting parameters
        xlimits = [-1 2.5];
        ylimits = [-1.5 2];
        zlimits = [0 6];
        tick_quantum = 0.5;
        quiverScaler = 1;
        azimuth = 0;
        elevation = 90;
       
    case 'UR3'
        %% UR3 Robot
        % Link lengths:
        L1 = 152; L2 = 244; L3 = 213; L4 = 83; L5 = 83; L6 = 82; %in mm
        D1 = 120; D2 = 93; D3 = 83; D4 = 82; aff =L6;
        aff = L6-40;
        
        % Start angles
        thetalist0 = zeros([10 1]);
        thetalist0(4) = pi/4;
        
        % Screw axis locations from base frame in home position
        q(:,1) = [0 0 0]';% first joint, base frame
        q(:,2) = [0 0 L1]';
        q(:,3) = [0 -D1 L1+L2]';
        q(:,4) = [0 -(D1-D2) L1+L2+L3]';
        q(:,5) = [0 -(D1-D2+L5) L1+L2+L3]';
        q(:,6) = [0 -(D1-D2+L5) L1+L2+L3+L4]';
        q(:,7) = [0 -(D1-D2+L5+L6) L1+L2+L3+L4]';% Imaginary joint
        q(:,8) = [0 -(D1-D2+L5+L6) L1+L2+L3+L4]';% Imaginary joint
        q(:,9) = [0 -(D1-D2+L5+L6) L1+L2+L3+L4]';% Imaginary joint
        q(:,10) = [0 -(D1-D2+L5+L6+aff) L1+L2+L3+L4]'; % location of affordance frame
        q(:,11) = [0 0 0]';% Closed-loop end-effector
        
        
        % Type and alignment of screw axes
        w(:,1) = [0 0 1]';
        w(:,2) = [0 -1 0]'; 
        w(:,3) = [0 -1 0]'; 
        w(:,4) = [0 -1 0]'; 
        w(:,5) = [0 0 1]'; 
        w(:,6) = [0 -1 0]'; 
        w(:,7) = [1 0 0]'; % Imaginary joint
        w(:,8) = [0 1 0]'; % Imaginary joint
        w(:,9) = [0 0 1]'; % Imaginary joint
        w(:,10) = [1 0 0]'; % affordance
        
        % Set the first frame as identity
        mlist(:,:,1) = eye(4);
        
        % Construct screw axes and frames
        for i = 1:length(w)
        v(:,i) = -cross(w(:,i), q(:,i));
        slist(:,i) = [w(:,i); v(:,i)];
        mlist(:,:, i+1) = tM('z', 0, q(:,i+1)); % End of each link, including the imaginary one
        end

        % % Affordance type
        % if strcmpi(affType,'pure_trans')
        %     slist(:,end) = [1 0 0 0 0 0]';
        % end

        % Closure error - Define as identity in this configuration
        Tsd = eye(4);

        % Task indices
        x1Tindex = 10;
        x2Tindex = 9;

        % Plotting parameters
        xlimits = [-500 500];
        ylimits = [-500 500];
        zlimits = [-100 900];
        tick_quantum = 50;
        quiverScaler = 200;
        azimuth = 45;
        elevation = 45;

     case 'UR5'
        %Link lengths
        mconv = 1000;
        W1 = 109/mconv; W2 = 82/mconv; L1 = 425/mconv; L2 = 392/mconv; H1 = 89/mconv; H2 = 95/mconv; W3 = 135.85/mconv; W4 = 119.7/mconv; W6 = 93/mconv; aff = 100/mconv;
        
        % Start angles
        thetalist0 = zeros([10 1]);
        
        % Screw axis locations from base frame in home position
        q(:,1) = [0 0 H1]';% first joint, base frame
        q(:,2) = [0 W3 H1]';
        q(:,3) = [L1 W3-W4 H1]';
        q(:,4) = [L1+L2 W3-W4 H1]';
        q(:,5) = [L1+L2 W3-W4+W6 H1]';
        q(:,6) = [L1+L2 W3-W4+W6 H1-H2]';
        q(:,7) = [L1+L2 W3-W4+W6+W2 H1-H2]';% Imaginary joint
        q(:,8) = [L1+L2 W3-W4+W6+W2 H1-H2]';% Imaginary joint
        q(:,9) = [L1+L2 W3-W4+W6+W2 H1-H2]';% Imaginary joint
        q(:,10) = [L1+L2 W3-W4+W6+W2+aff H1-H2]'; % location of affordance frame
        q(:,11) = [0 0 0]';% Closed-loop end-effector

    
        % Type and alignment of screw axes
        w(:,1) = [0 0 1]';
        w(:,2) = [0 1 0]'; 
        w(:,3) = [0 1 0]'; 
        w(:,4) = [0 1 0]'; 
        w(:,5) = [0 0 -1]'; 
        w(:,6) = [0 1 0]'; 
        w(:,7) = [1 0 0]'; % Imaginary joint
        w(:,8) = [0 1 0]'; % Imaginary joint
        w(:,9) = [0 0 1]'; % Imaginary joint
        w(:,10) = [1 0 0]'; % affordance

       % Set the first frame as identity
        mlist(:,:,1) = eye(4);
        
        % Construct screw axes and frames
        for i = 1:length(w)
        v(:,i) = -cross(w(:,i), q(:,i));
        slist(:,i) = [w(:,i); v(:,i)];
        mlist(:,:, i+1) = tM('z', 0, q(:,i+1)); % End of each link, including the imaginary one
        end

        % % Affordance type
        % if strcmpi(affType,'pure_trans')
        %     slist(:,end) = [1 0 0 0 0 0]';
        % end

        % Closure error - Define as identity in this configuration
        Tsd = eye(4);

        % Task indices
        x1Tindex = 10;
        x2Tindex = 9;

         % Plotting parameters
        xlimits = [-.2 1.0];
        ylimits = [-.2 1.0];
        zlimits = [-.7 .5];
        tick_quantum = 0.050;
        quiverScaler = 0.1;
        azimuth = 45;
        elevation = 45;

    otherwise
        display("Invalid robot type")

end