function [pos, rot, rpy] = getEEPose(theta)
    [~,~,~,~,~,T06] = forwardKinematics(theta);
    
    rot = T06(1:3,1:3);
    
    % EE position - transform the EE center
    ee_zero = [15.522; 3.015; 1.244];
    pos = T06 * [ee_zero; 1];
    pos = pos(1:3);
    
    % Robust RPY
    if abs(rot(3,1)) < 1 - 1e-10
        pitch = asin(-rot(3,1));
        roll  = atan2(rot(3,2)/cos(pitch), rot(3,3)/cos(pitch));
        yaw   = atan2(rot(2,1)/cos(pitch), rot(1,1)/cos(pitch));
    else
        yaw = 0;
        if rot(3,1) < 0
            pitch = pi/2;
            roll  = atan2(rot(1,2), rot(1,3));
        else
            pitch = -pi/2;
            roll  = atan2(-rot(1,2), -rot(1,3));
        end
    end
    
    rpy = rad2deg([roll; pitch; yaw]);
end