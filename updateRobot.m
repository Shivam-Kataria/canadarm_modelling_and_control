function updateRobot(LinkPatches, V0, theta)
    [T01,T02,T03,T04,T05,T06] = forwardKinematics(theta);
    [T01_0,T02_0,T03_0,T04_0,T05_0,T06_0] = forwardKinematics([0,0,0,0,0,0]);

    % T07 = T06 for link 7 (end effector, moves with joint 6)
    T07   = T06;
    T07_0 = T06_0;

    Tlist   = {T01,   T02,   T03,   T04,   T05,   T06,   T07  };
    Tlist_0 = {T01_0, T02_0, T03_0, T04_0, T05_0, T06_0, T07_0};

    for i = 1:length(LinkPatches)
        T_current = Tlist{i};
        T_initial = Tlist_0{i};
        T_rel = T_current / T_initial;

        V_h   = [V0{i}(:,1:3), ones(size(V0{i},1),1)]';
        V_new = T_rel * V_h;
        set(LinkPatches{i}, 'Vertices', V_new(1:3,:)');
    end

    drawnow limitrate;
end