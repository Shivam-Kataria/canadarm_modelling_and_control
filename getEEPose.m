function [pos, euler] = getEEPose(theta)
    [~,~,~,~,~,T06] = forwardKinematics(theta);

    pos = T06(1:3,4);
    R   = T06(1:3,1:3);

    theta_e = atan2d(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));

    if abs(sind(theta_e)) > 1e-6
        phi = atan2d(R(2,3), R(1,3));
        psi = atan2d(R(3,2), -R(3,1));
    else
        phi = 0;
        psi = atan2d(-R(1,2), R(1,1));
    end

    euler = [phi, theta_e, psi];
end