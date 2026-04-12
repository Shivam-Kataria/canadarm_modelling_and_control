function [pos, rot, rpy] = getEEPose(theta)
    [~,~,~,~,~,T06] = forwardKinematics(theta);
    pos = T06(1:3, 4);
    rot = T06(1:3, 1:3);
    
    % Extract roll, pitch, yaw from rotation matrix
    pitch = atan2(-rot(3,1), sqrt(rot(1,1)^2 + rot(2,1)^2));
    yaw   = atan2(rot(2,1)/cos(pitch), rot(1,1)/cos(pitch));
    roll  = atan2(rot(3,2)/cos(pitch), rot(3,3)/cos(pitch));
    
    rpy = rad2deg([roll; pitch; yaw]);
end