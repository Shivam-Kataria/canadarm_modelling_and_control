function [pos, rot, rpy] = getEEPose(theta)
    % Use same joint data as updateRobot
    origins = [0.4500  0.0000  1.2446;
               0.7206  0.3986  1.2455;
               7.7567  0.9602  1.2461;
               14.7132 1.4514  1.2463;
               15.0490 1.9533  1.2452;
               15.5100 2.6372  1.2464];

    axes = [1  0  0;
            0  1  0;
            0  1  0;
            0  1  0;
            1  0  0;
            0  1  0];

    % Start with EE center at zero position
    ee_pos = [15.522; 3.015; 1.244];
    
    % Track cumulative rotation
    R_total = eye(3);
    cur_origins = origins;

    % Apply each joint sequentially - same as updateRobot
    for j = 1:6
        origin = cur_origins(j,:)';
        ax = axes(j,:)';
        angle = deg2rad(theta(j));
        
        c = cos(angle); s = sin(angle);
        K = [0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0];
        R = eye(3) + s*K + (1-c)*K*K;
        
        % Rotate EE position about this joint
        ee_pos = R*(ee_pos - origin) + origin;
        
        % Update downstream origins
        for k = j+1:6
            cur_origins(k,:) = (R*(cur_origins(k,:)' - origin) + origin)';
        end
        
        % Update axes
        for k = j+1:6
            axes(k,:) = (R*axes(k,:)')';
        end
        
        R_total = R * R_total;
    end

    pos = ee_pos;
    rot = R_total;
    
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