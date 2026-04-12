% testRobotCAD.m
disp('Loading robot CAD data...');
[V0, F, C, numLinks] = loadRobotData('RobotName_LinksData.mat');
fprintf('Loaded %d links\n', numLinks);

% Keep clean original copy
V0_orig = V0;

% Draw robot
fig = figure;
ax = axes(fig);
LinkPatches = drawRobot(ax, V0_orig, F, C, numLinks);
disp('Robot drawn at zero position');
pause(2);

% Test joint 1 only first
disp('Testing joint 1 at 30 deg...');
updateRobot(LinkPatches, V0_orig, [30,0,0,0,0,0]);
pause(2);

disp('Testing joint 1 at 60 deg...');
updateRobot(LinkPatches, V0_orig, [60,0,0,0,0,0]);
pause(2);

% Reset to zero
disp('Back to zero...');
updateRobot(LinkPatches, V0_orig, [0,0,0,0,0,0]);
pause(2);

% Animate each joint one at a time so you can see them individually

disp('Joint 1 sweep...');
for t = 0:2:60
    if ~isvalid(LinkPatches{1}), break; end
    updateRobot(LinkPatches, V0_orig, [t,0,0,0,0,0]);
    title(ax, sprintf('Joint 1 = %d deg', t));
    pause(0.05);
end
pause(1);

disp('Joint 2 sweep...');
for t = 0:2:60
    if ~isvalid(LinkPatches{1}), break; end
    updateRobot(LinkPatches, V0_orig, [0,t,0,0,0,0]);
    title(ax, sprintf('Joint 2 = %d deg', t));
    pause(0.05);
end
pause(1);

disp('Joint 3 sweep...');
for t = 0:2:60
    if ~isvalid(LinkPatches{1}), break; end
    updateRobot(LinkPatches, V0_orig, [0,0,t,0,0,0]);
    title(ax, sprintf('Joint 3 = %d deg', t));
    pause(0.05);
end
pause(1);

disp('Joint 4 sweep...');
for t = 0:2:60
    if ~isvalid(LinkPatches{1}), break; end
    updateRobot(LinkPatches, V0_orig, [0,0,0,t,0,0]);
    title(ax, sprintf('Joint 4 = %d deg', t));
    pause(0.05);
end
pause(1);

disp('Joint 5 sweep...');
for t = 0:2:60
    if ~isvalid(LinkPatches{1}), break; end
    updateRobot(LinkPatches, V0_orig, [0,0,0,0,t,0]);
    title(ax, sprintf('Joint 5 = %d deg', t));
    pause(0.05);
end
pause(1);

disp('Joint 6 sweep...');
for t = 0:2:60
    if ~isvalid(LinkPatches{1}), break; end
    updateRobot(LinkPatches, V0_orig, [0,0,0,0,0,t]);
    title(ax, sprintf('Joint 6 = %d deg', t));
    pause(0.05);
end