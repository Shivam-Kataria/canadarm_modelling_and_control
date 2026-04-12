function theta = inverseKinematics(p_target, theta_init)
    theta   = theta_init;
    maxIter = 100;
    tol     = 0.001;
    lam     = 0.01;

    for iter = 1:maxIter
        [~,~,~,~,~,T06] = forwardKinematics(theta);
        p_cur = T06(1:3,4);
        err   = p_target - p_cur;

        if norm(err) < tol
            break;
        end

        J   = zeros(3,6);
        dth = 0.0001;
        for j = 1:6
            th2    = theta;
            th2(j) = th2(j) + rad2deg(dth);
            [~,~,~,~,~,T2] = forwardKinematics(th2);
            J(:,j) = (T2(1:3,4) - p_cur) / dth;
        end

        dtheta = J' / (J*J' + lam^2*eye(3)) * err;
        theta  = theta + rad2deg(dtheta)';
        theta  = max(-180, min(180, theta));
    end
end