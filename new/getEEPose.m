function [pos, rot, rpy] = getEEPose(theta)
    [~,~,~,~,~,T06] = forwardKinematics(theta);
    rot = T06(1:3, 1:3);
    
    % Actual EE position with CAD offset
    offset_dh = [-0.341; -0.736; 3.015];
    pos = T06(1:3, 4) + rot * offset_dh;
    
    % Robust ZYX euler angles (yaw, pitch, roll)
    if abs(rot(3,1)) < 1 - 1e-10
        pitch = asin(-rot(3,1));
        roll  = atan2(rot(3,2)/cos(pitch), rot(3,3)/cos(pitch));
        yaw   = atan2(rot(2,1)/cos(pitch), rot(1,1)/cos(pitch));
    else
        % Gimbal lock
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