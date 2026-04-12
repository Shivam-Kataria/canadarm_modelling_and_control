function updateRobot(LinkPatches, V0, theta)

    origins = [0.4500  -0.0000  1.2446;
               0.7206   0.3986  1.2455;
               7.7567   0.9602  1.2461;
               14.7132  1.4514  1.2463;
               15.0490  1.9533  1.2452;
               15.5100  2.6372  1.2464];

    axes = [1  0  0;
            0  1  0;
            0  1  0;
            0  1  0;
            1  0  0;
            0  1  0];

    V_current = cell(1,7);
    for i = 1:7
        V_current{i} = V0{i}(:,1:3);
    end

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

        % Update downstream joint origins AND axes
        for k = j+1:6
            % Update origin
            v = origins(k,:) - origin;
            origins(k,:) = v*cos(angle) + cross(ax,v)*sin(angle) + ax*dot(ax,v)*(1-cos(angle)) + origin;

            % Update axis direction
            ax_k = axes(k,:);
            axes(k,:) = ax_k*cos(angle) + cross(ax,ax_k)*sin(angle) + ax*dot(ax,ax_k)*(1-cos(angle));
        end
    end

    for i = 1:7
        set(LinkPatches{i}, 'Vertices', V_current{i});
    end

    drawnow limitrate;
end