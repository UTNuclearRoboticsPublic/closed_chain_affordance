function screwPathT = screwPathCreator(S, screwPathTstart, delta_theta, iterations)
%% Given a screw axis and a moving frame, creates a matrix of discretized points on the screw path
    ri = 1;  %plotting matrix row indexer as well as loop incrementer
    initialScrewPathTStart = screwPathTstart;
    while ri<=iterations

%         % Reverse the stepping direction at half iterations
%         if ri==iterations/2
%             delta_theta = -delta_theta;
%             screwPathTstart = initialScrewPathTStart;
%         end
%         
%         if ri>= iterations/2
%             screwPathT(:,:,2:end+1) = screwPathT(:,:,1:end)
%             screwPathT(:,:,1) = MatrixExp6(VecTose3(S)*delta_theta) * screwPathTstart;
%             screwPathTstart = screwPathT(:,:,1);
%         end
        screwPathT(:,:,ri) = MatrixExp6(VecTose3(S)*delta_theta) * screwPathTstart;
        screwPathTstart = screwPathT(:,:,ri); % Stepped screw path TF will be the start TF for the next step
        ri = ri + 1;
    end
end