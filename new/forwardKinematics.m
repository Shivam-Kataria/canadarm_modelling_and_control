function [T01,T02,T03,T04,T05,T06] = forwardKinematics(theta)
    % Joint origins and axes derived directly from CAD geometry
    % These match exactly what updateRobot uses
    
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

    % Build cumulative transforms
    T = eye(4);
    Tall = cell(1,6);
    
    for j = 1:6
        origin = origins(j,:)';
        ax = axes(j,:)';
        angle = deg2rad(theta(j));
        
        % Rotation matrix about axis
        c = cos(angle); s = sin(angle);
        K = [0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0];
        R = eye(3) + s*K + (1-c)*K*K;
        
        % Transform for this joint
        Tj = eye(4);
        Tj(1:3,1:3) = R;
        Tj(1:3,4) = origin - R*origin;
        
        T = Tj * T;
        Tall{j} = T;
    end
    
    T01 = Tall{1};
    T02 = Tall{2};
    T03 = Tall{3};
    T04 = Tall{4};
    T05 = Tall{5};
    T06 = Tall{6};
end