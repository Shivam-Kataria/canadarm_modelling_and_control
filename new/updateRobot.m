function updateRobot(LinkPatches, V0, theta)

    % Joint origins from actual CAD geometry
    origins = [0.4500  -0.0000  1.2446;   % joint 1
               0.7206   0.3986  1.2455;   % joint 2
               7.7567   0.9602  1.2461;   % joint 3
               14.7132  1.4514  1.2463;   % joint 4
               15.0490  1.9533  1.2452;   % joint 5
               15.5100  2.6372  1.2464];  % joint 6

    % Joint rotation axes (verified from FK)
    axes = [1  0  0;   % joint 1 - Y axis (was Z, wrong)
            0  1  0;   % joint 2 - Y axis (correct)
            0  1  0;   % joint 3 - Y axis (was Z, wrong)
            0  1  0;   % joint 4 - Y axis (was Z, wrong)
            1  0  0;   % joint 5 - X axis (was Y, wrong)
            0  1  0];   % joint 6 - Y axis (was Z, wrong)

    % Copy vertices
    V_current = cell(1,7);
    for i = 1:7
        V_current{i} = V0{i}(:,1:3);
    end

    % Apply joints sequentially
    for j = 1:6
        angle = deg2rad(theta(j));
        origin = origins(j,:);
        ax = axes(j,:);

        % Rotate all links from j+1 onward
        for i = j+1:7
            V = V_current{i};
            V_new = zeros(size(V));
            for n = 1:size(V,1)
                v = V(n,:) - origin;
                v_rot = v*cos(angle) + cross(ax,v)*sin(angle) + ax*dot(ax,v)*(1-cos(angle));
                V_new(n,:) = v_rot + origin;
            end
            V_current{i} = V_new;
        end

        % Update downstream joint origins
        for k = j+1:6
            v = origins(k,:) - origin;
            origins(k,:) = v*cos(angle) + cross(ax,v)*sin(angle) + ax*dot(ax,v)*(1-cos(angle)) + origin;
        end
    end

    % Update patches
    for i = 1:7
        set(LinkPatches{i}, 'Vertices', V_current{i});
    end

    drawnow limitrate;
end