function robotSim()
% robotSim - Main simulation engine for PUMA560 robot
% ENGR 486 - called by the GUI to do all FK/IK math and rendering
% 
% Your friend's GUI should call these functions:
%   updateRobot(LinkPatches, V0, theta)
%   [pos, euler] = getEEPose(theta)
%   [T01,T02,T03,T04,T05,T06] = forwardKinematics(theta)
%   theta = inverseKinematics(target_pos, theta_init)
%   [points] = generateTrajectory(p_start, p_end, N, type)
end

% =========================================================================
% DH TRANSFORMATION MATRIX
% =========================================================================
function T = DHmat(a, d, alpha, theta)
    alpha = deg2rad(alpha);
    theta = deg2rad(theta);
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

% =========================================================================
% FORWARD KINEMATICS
% Input:  theta = [t1,t2,t3,t4,t5,t6] in degrees (joint angles only)
% Output: T01 through T06 = cumulative homogeneous transforms
% =========================================================================
function [T01,T02,T03,T04,T05,T06] = forwardKinematics(theta)
    % YOUR DH parameters from your table
    DH_a     = [0,     0,     7.37,  0,     0,     0    ];  % meters
    DH_d     = [0,     0.722, 0.508, 0,     7.04,  0.731];  % meters
    DH_alpha = [-90,   90,    0,     -90,   90,    -90  ];  % degrees
    DH_off   = [90,   -90,   -90,   -90,    0,     0    ];  % theta offsets from DH table

    % Add DH offsets to joint angles
    th = theta + DH_off;

    % Individual link transforms
    T01 = DHmat(DH_a(1), DH_d(1), DH_alpha(1), th(1));
    T12 = DHmat(DH_a(2), DH_d(2), DH_alpha(2), th(2));
    T23 = DHmat(DH_a(3), DH_d(3), DH_alpha(3), th(3));
    T34 = DHmat(DH_a(4), DH_d(4), DH_alpha(4), th(4));
    T45 = DHmat(DH_a(5), DH_d(5), DH_alpha(5), th(5));
    T56 = DHmat(DH_a(6), DH_d(6), DH_alpha(6), th(6));

    % Cumulative transforms (world to each link)
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    T06 = T05 * T56;
end

% =========================================================================
% UPDATE ROBOT VISUAL
% Applies FK transforms to patch vertices so the 3D model moves
% Input:  LinkPatches = cell array {L1,L2,...,L6} of patch handles from GUI
%         V0          = cell array {V1,V2,...,V6} of original n×4 vertices
%         theta       = [t1,t2,t3,t4,t5,t6] in degrees
% =========================================================================
function updateRobot(LinkPatches, V0, theta)
    [T01,T02,T03,T04,T05,T06] = forwardKinematics(theta);

    % T for each link: link i moves with frame i
    Tlist = {eye(4), T01, T02, T03, T04, T05, T06};

    for i = 1:6
        T    = Tlist{i+1};
        Vnew = (T * V0{i}')';           % rotate/translate all vertices
        set(LinkPatches{i}, 'Vertices', Vnew(:,1:3));
    end

    drawnow limitrate;
end

% =========================================================================
% GET END-EFFECTOR POSE
% Input:  theta = [t1,t2,t3,t4,t5,t6] in degrees
% Output: pos   = [x; y; z] in meters
%         euler = [phi, theta, psi] ZYZ Euler angles in degrees
% =========================================================================
function [pos, euler] = getEEPose(theta)
    [~,~,~,~,~,T06] = forwardKinematics(theta);

    % Position
    pos = T06(1:3, 4);

    % Rotation matrix
    R = T06(1:3, 1:3);

    % ZYZ Euler angles (as required by guideline)
    theta_e = atan2d(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));

    if abs(sind(theta_e)) > 1e-6
        phi = atan2d(R(2,3), R(1,3));
        psi = atan2d(R(3,2), -R(3,1));
    else
        % Gimbal lock case
        phi = 0;
        psi = atan2d(-R(1,2), R(1,1));
    end

    euler = [phi, theta_e, psi];  % degrees, ZYZ convention
end

% =========================================================================
% GENERATE TRAJECTORY POINTS
% Input:  p_start = [x;y;z] start position in meters
%         p_end   = [x;y;z] end position in meters
%         N       = number of steps
%         type    = 'Linear' or 'Circular'
% Output: points  = N×3 matrix of trajectory waypoints
% =========================================================================
function points = generateTrajectory(p_start, p_end, N, type)
    points = zeros(N, 3);

    if strcmp(type, 'Linear')
        for i = 1:N
            s = i / N;
            points(i,:) = (p_start + s*(p_end - p_start))';
        end

    elseif strcmp(type, 'Circular')
        % Circle in XY plane centered between start and end
        center = (p_start + p_end) / 2;
        r      = norm(p_end(1:2) - p_start(1:2)) / 2;
        for i = 1:N
            angle      = 2*pi * i/N;
            points(i,1) = center(1) + r*cos(angle);
            points(i,2) = center(2) + r*sin(angle);
            points(i,3) = p_start(3) + (i/N)*(p_end(3)-p_start(3));
        end
    end
end

% =========================================================================
% INVERSE KINEMATICS (Numerical - Jacobian Pseudoinverse)
% Replace the inner loop with your analytic IK from class when ready
% Input:  p_target   = [x;y;z] desired EE position in meters
%         theta_init = [t1,...,t6] starting joint angles in degrees
% Output: theta      = [t1,...,t6] solved joint angles in degrees
% =========================================================================
function theta = inverseKinematics(p_target, theta_init)
    theta   = theta_init;
    maxIter = 100;
    tol     = 0.001;   % 1mm tolerance
    lam     = 0.01;    % damping factor

    for iter = 1:maxIter
        [~,~,~,~,~,T06] = forwardKinematics(theta);
        p_cur = T06(1:3,4);
        err   = p_target - p_cur;

        if norm(err) < tol
            break;
        end

        % Numerical Jacobian (3×6, position only)
        J   = zeros(3,6);
        dth = 0.0001;  % radians
        for j = 1:6
            th2    = theta;
            th2(j) = th2(j) + rad2deg(dth);
            [~,~,~,~,~,T2] = forwardKinematics(th2);
            J(:,j) = (T2(1:3,4) - p_cur) / dth;
        end

        % Damped least squares step
        dtheta = J' / (J*J' + lam^2*eye(3)) * err;
        theta  = theta + rad2deg(dtheta)';

        % Clamp to joint limits
        theta = max(-180, min(180, theta));
    end
end

% =========================================================================
% LOAD ROBOT CAD DATA
% Call this once at startup to load your MAT file
% Input:  filepath = full path to your RobotName_LinksData.mat
% Output: V0       = cell array of original vertices per link (n×4)
%         F        = cell array of faces per link
%         C        = cell array of colors per link
%         numLinks = number of links found
% =========================================================================
function [V0, F, C, numLinks] = loadRobotData(filepath)
    data   = load(filepath);
    fields = fieldnames(data);

    % Find struct fields named s1, s2, ...
    linkFields = {};
    for i = 1:length(fields)
        if startsWith(fields{i},'s') && isstruct(data.(fields{i}))
            linkFields{end+1} = fields{i};
        end
    end
    linkFields = sort(linkFields);
    numLinks   = min(length(linkFields), 6);

    V0 = cell(1, numLinks);
    F  = cell(1, numLinks);
    C  = cell(1, numLinks);

    for i = 1:numLinks
        s  = data.(linkFields{i});
        sf = fieldnames(s);

        vf     = sf(startsWith(sf,'V')); V0{i} = s.(vf{1});
        ff     = sf(startsWith(sf,'F')); F{i}  = s.(ff{1});
        cf     = sf(startsWith(sf,'C')); C{i}  = s.(cf{1});

        % Ensure vertices are n×4 for homogeneous transforms
        if size(V0{i}, 2) == 3
            V0{i} = [V0{i}, ones(size(V0{i},1), 1)];
        end
    end
end

% =========================================================================
% DRAW INITIAL ROBOT (patch objects)
% Call once after loading data to create patch objects in the axes
% Input:  ax       = the uiaxes handle from GUI
%         V0, F, C = from loadRobotData
%         numLinks = from loadRobotData
% Output: LinkPatches = cell array of patch handles, pass back to updateRobot
% =========================================================================
function LinkPatches = drawRobot(ax, V0, F, C, numLinks)
    cla(ax);
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    view(ax, 35, 25);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
    ax.Color     = [0.10 0.10 0.13];
    ax.GridColor = [0.30 0.30 0.30];
    ax.XColor    = [0.70 0.70 0.70];
    ax.YColor    = [0.70 0.70 0.70];
    ax.ZColor    = [0.70 0.70 0.70];
    title(ax, 'PUMA560 Robot Simulator', ...
        'Color',[0.9 0.9 0.9], 'FontSize',11, 'FontWeight','bold');

    colors = [0.45 0.45 0.50;
              0.20 0.45 0.72;
              0.20 0.45 0.72;
              0.85 0.33 0.10;
              0.85 0.33 0.10;
              0.15 0.68 0.38];

    LinkPatches = cell(1, numLinks);
    for i = 1:numLinks
        LinkPatches{i} = patch(ax, ...
            'Faces',           F{i}, ...
            'Vertices',        V0{i}(:,1:3), ...
            'FaceColor',       colors(min(i,6),:), ...
            'EdgeColor',       'none', ...
            'FaceLighting',    'gouraud', ...
            'AmbientStrength', 0.4, ...
            'DiffuseStrength', 0.7);
    end

    light(ax, 'Position', [1  1  2], 'Style','infinite');
    light(ax, 'Position', [-1 -1 1], 'Style','infinite');
end

% =========================================================================
% DRAW SKELETON (no CAD data - just joint origins connected by lines)
% Useful for testing FK before CAD is loaded
% =========================================================================
function drawSkeleton(ax, theta)
    cla(ax);
    hold(ax,'on');
    axis(ax,'equal'); grid(ax,'on');
    view(ax,35,25);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
    ax.Color = [0.10 0.10 0.13];
    ax.GridColor = [0.30 0.30 0.30];
    ax.XColor = [0.70 0.70 0.70];
    ax.YColor = [0.70 0.70 0.70];
    ax.ZColor = [0.70 0.70 0.70];
    title(ax,'PUMA560 Skeleton','Color',[0.9 0.9 0.9],'FontSize',11,'FontWeight','bold');

    [T01,T02,T03,T04,T05,T06] = forwardKinematics(theta);
    Tlist = {eye(4), T01, T02, T03, T04, T05, T06};

    origins = zeros(7,3);
    for i = 1:7
        origins(i,:) = Tlist{i}(1:3,4)';
    end

    plot3(ax, origins(:,1), origins(:,2), origins(:,3), ...
        'o-', 'Color',[0.20 0.65 1.0], 'LineWidth',3, ...
        'MarkerFaceColor',[1 0.5 0.1], 'MarkerSize',8);

    % World frame axes
    sc = 0.5;
    quiver3(ax,0,0,0,sc,0,0,'r','LineWidth',2,'MaxHeadSize',0.5);
    quiver3(ax,0,0,0,0,sc,0,'g','LineWidth',2,'MaxHeadSize',0.5);
    quiver3(ax,0,0,0,0,0,sc,'b','LineWidth',2,'MaxHeadSize',0.5);
    text(ax,sc,0,0,'X','Color','r','FontWeight','bold');
    text(ax,0,sc,0,'Y','Color','g','FontWeight','bold');
    text(ax,0,0,sc,'Z','Color','b','FontWeight','bold');
end