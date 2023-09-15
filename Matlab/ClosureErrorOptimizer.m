function [qp, qsb, errTwist] = ClosureErrorOptimizer(mErr, slist, qp, qsb, Np, Ns, Tsd, taskOffset)

    thetalist =[qp; qsb]; 
    
    % Calculate the closure error    
    Tse = FKinSpace(mErr,slist,thetalist); %FK
    errTwist = Adjoint(Tse)*se3ToVec(MatrixLog6(TransInv(Tse)*Tsd));
    
    % Adjust qp and qs based on this error
    N = [Np Ns];
    q = pinv(N)*(errTwist);
    qp = qp+q(1:end-taskOffset);
    qsb = qsb+q(end-taskOffset+1:end);
    
    % Update thetalist
    thetalist =[qp; qsb]; 
    
    % Compute final error
    Tse = FKinSpace(mErr,slist,thetalist); %FK
    errTwist = Adjoint(Tse)*se3ToVec(MatrixLog6(TransInv(Tse)*Tsd));

end