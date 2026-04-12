[V0, F, C, numLinks] = loadRobotData('RobotName_LinksData.mat');
V0_orig = V0;

fig = figure;
ax = axes(fig);
LinkPatches = drawRobot(ax, V0_orig, F, numLinks);
pause(1);

disp('Testing joint 2 at 30 deg...');
updateRobot(LinkPatches, V0_orig, [0,30,0,0,0,0]);
pause(10);

disp('Testing joint 3 at 30 deg...');
updateRobot(LinkPatches, V0_orig, [0,0,30,0,0,0]);