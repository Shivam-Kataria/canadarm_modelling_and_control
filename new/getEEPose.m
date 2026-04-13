function [pos, rot, rpy] = getEEPose(theta)
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

    ee_pos = [15.522; 3.015; 1.244];
    R_total = eye(3);
    cur_origins = origins;
    cur_axes = axes;

    for j = 1:6
        origin = cur_origins(j,:)';
        ax = cur_axes(j,:)';
        angle = deg2rad(theta(j));

        c = cos(angle); s = sin(angle);
        K = [0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0];
        R = eye(3) + s*K + (1-c)*K*K;

        ee_pos = R*(ee_pos - origin) + origin;

        for k = j+1:6
            cur_origins(k,:) = (R*(cur_origins(k,:)' - origin) + origin)';
            cur_axes(k,:) = (R*cur_axes(k,:)')';
        end

        R_total = R * R_total;
    end

    pos = ee_pos;
    rot = R_total;

    % Extract RPY from rotation matrix columns
    % X axis of EE frame
    x_axis = rot(:,1);
    y_axis = rot(:,2);
    z_axis = rot(:,3);

    % Yaw: rotation about Z (world)
    yaw = atan2(x_axis(2), x_axis(1));
    
    % Pitch: rotation about Y (world)
    pitch = atan2(-x_axis(3), sqrt(x_axis(1)^2 + x_axis(2)^2));
    
    % Roll: rotation about X (world)
    roll = atan2(y_axis(3), z_axis(3));

    rpy = rad2deg([roll; pitch; yaw]);
end