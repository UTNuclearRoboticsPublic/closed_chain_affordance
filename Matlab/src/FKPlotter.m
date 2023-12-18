function [plotrepf, plotrepl, plotrepj, plotrept, plotrepn]  = FKPlotter(mlist,slist,thetalist, x1Tindex, x2Tindex, xlimits, ylimits, zlimits, tick_quantum, quiverScaler,  azimuth, elevation, robotType, robot, screwPathMatrix)

        for i = 1:size(slist,2)
            FK(:,:,i,1) = FKinSpace(mlist(:,:, i+1), slist(:,1:i), thetalist(1:i)); % Starting guess for all relevant frames/tasks
        end

%             figure(3)
            daspect([1 1 1]);
        
            title('Manipulator Configuration','FontSize', 15);
            view(azimuth,elevation);
        
            
            % axis limits
            xlim(xlimits);
            ylim(ylimits);
            zlim(zlimits)
        
            % grid ticks
            xticks(xlimits(1):tick_quantum:xlimits(2));
            yticks(ylimits(1):tick_quantum:ylimits(2));
            zticks(zlimits(1):tick_quantum:zlimits(2));
            
            % labels
            xlabel('x');
            ylabel('y');
            zlabel('z');

            axis square
            grid on

                
    %% Dynamic plots: manipulator and EE frames
    for i = 1:size(FK, 4)
        
        if strcmpi(robotType, 'UR5')
%             figure(3)
            targetJointPosition = thetalist(:,i);
            plotrepn(i,1) = show(robot,targetJointPosition(1:6), 'Frames','off');
            campos([3 -1 2])
            camzoom(2)
            camva(20) 
        else
            plotrepn(i,1) = 1;
        end
        hold on
        plot3(screwPathMatrix(:,1),screwPathMatrix(:,2), screwPathMatrix(:,3), 'b:', 'LineWidth', 2); 

        % Place a marker at the base joint
        plot3(0,0,0, 'o', 'MarkerSize', 15, 'MarkerFaceColor','m')
        % Extract data to plot
        % Link end point coordinates from base from to x2
        xLink = [0 reshape(FK(1, 4, 1:x2Tindex-1, i), x2Tindex-1, [])']; % x
        yLink = [0 reshape(FK(2, 4, 1:x2Tindex-1, i), x2Tindex-1, [])']; % y
        zLink = [0 reshape(FK(3, 4, 1:x2Tindex-1, i), x2Tindex-1, [])']; % z

        % Link end point coordinates from x2 to x1, making up the imaginary
        % link
        imXLink = [reshape(FK(1, 4, x2Tindex-1:x1Tindex, i), 1, [])']; % x
        imYLink = [reshape(FK(2, 4, x2Tindex-1:x1Tindex, i), 1, [])']; % y
        imZLink = [reshape(FK(3, 4, x2Tindex-1:x1Tindex, i), 1, [])']; % z

        % Task orientation and coordinates
        x1Rot = FK(1:3, 1:3, x1Tindex, i); % 2x2 rotation matrix of the first task
        x1Pos = FK(1:3, 4, x1Tindex, i); % position part of the first task
        x2Rot = FK(1:3, 1:3, x2Tindex, i); % 2x2 rotation matrix of the second task
        x2Pos = FK(1:3, 4, x2Tindex, i); % position part of the second task
        x2PosTraj(i,1:3) = FK(1:3, 4, x2Tindex, i); % position part of the second task storing to remember last points for connecting while plotting trajectory
        
        %% Plot
       
        % Task frames
        plotrepf(i,1) = quiver3(x1Pos(1), x1Pos(2), x1Pos(3), x1Rot(1,1), x1Rot(2,1), x1Rot(3,1), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScaleFactor',quiverScaler);% x1 task frame x vector
        plotrepf(i,2) = quiver3(x1Pos(1), x1Pos(2), x1Pos(3), x1Rot(1,2), x1Rot(2,2), x1Rot(3,2),'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScaleFactor',quiverScaler); % x1 task frame y vector
        plotrepf(i,3) = quiver3(x1Pos(1), x1Pos(2), x1Pos(3), x1Rot(1,3), x1Rot(2,3), x1Rot(3,3),'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScaleFactor',quiverScaler); % x1 task frame z vector
        plotrepf(i,4) = quiver3(x2Pos(1), x2Pos(2), x2Pos(3), x2Rot(1,1), x2Rot(2,1), x2Rot(3,1),'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScaleFactor',quiverScaler); % x2 task frame x vector
        plotrepf(i,5) = quiver3(x2Pos(1), x2Pos(2), x2Pos(3), x2Rot(1,2), x2Rot(2,2), x2Rot(3,2),'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScaleFactor',quiverScaler); % x2 task frame y vector
        plotrepf(i,6) = quiver3(x2Pos(1), x2Pos(2), x2Pos(3), x2Rot(1,3), x2Rot(2,3), x2Rot(3,3),'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScaleFactor',quiverScaler); % x2 task frame z vector
        
        %Links
        % plotrepl(i,1) = plot3(xLink, yLink, zLink, 'k', 'LineWidth', 2); % real manipulator links
        plotrepl(i,2) = plot3(imXLink, imYLink, imZLink, 'b--', 'LineWidth', 2); % imaginary link connecting x1 and x2
       
        % Joint and task markers
        plotrepj(i,1) = plot3(xLink, yLink, zLink, 'o', 'MarkerSize', 10, 'MarkerFaceColor','m'); % all real joints
        plotrepj(i,2) = plot3(x2Pos(1), x2Pos(2), x2Pos(3), 'bd', 'MarkerSize', 12, 'MarkerFaceColor','m'); % x2 task marker
        plotrepj(i,3) = plot3(x1Pos(1), x1Pos(2), x1Pos(3), 'o', 'MarkerSize', 12, 'MarkerFaceColor','k'); % x1 task marker
        
        % Task 2 trajectory
        plotrept(i,1) = plot3(x2PosTraj(:, 1), x2PosTraj(:, 2), x2PosTraj(:, 3),  'm', 'LineWidth', 2); % x2 task marker
        

        % Draw, pause, and delete plot to create animation
%         drawnow;
%         pause(0.01);
%         if i~=size(FK, 4) % Don't delete the very last plot
%             delete(plotrepf);
%             delete(plotrepl);
%             delete(plotrepj);
%             delete(plotrept);
%             delete(plotrepn);
%         end
    
    end

end