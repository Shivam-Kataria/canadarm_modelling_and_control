% testRobot.m
% Run this to test the simulation engine without any GUI

%% TEST 1: Forward Kinematics at zero position
disp('--- TEST 1: FK at zero position ---');
theta_zero = [0, 0, 0, 0, 0, 0];
[T01,T02,T03,T04,T05,T06] = forwardKinematics(theta_zero);
disp('T06 (end-effector transform at zero position):');
disp(T06);

%% TEST 2: End-effector position and ZYZ euler angles at zero
disp('--- TEST 2: EE Pose at zero ---');
[pos, euler] = getEEPose(theta_zero);
fprintf('Position:      X=%.4f  Y=%.4f  Z=%.4f (m)\n', pos(1), pos(2), pos(3));
fprintf('ZYZ Euler:     phi=%.2f  theta=%.2f  psi=%.2f (deg)\n', euler(1), euler(2), euler(3));

%% TEST 3: FK at a non-zero pose
disp('--- TEST 3: FK at [30, 45, -30, 0, 60, 0] ---');
theta_test = [30, 45, -30, 0, 60, 0];
[pos2, euler2] = getEEPose(theta_test);
fprintf('Position:      X=%.4f  Y=%.4f  Z=%.4f (m)\n', pos2(1), pos2(2), pos2(3));
fprintf('ZYZ Euler:     phi=%.2f  theta=%.2f  psi=%.2f (deg)\n', euler2(1), euler2(2), euler2(3));

%% TEST 4: Skeleton visualization (no CAD needed)
disp('--- TEST 4: Drawing skeleton ---');
figure;
ax = axes;
drawSkeleton(ax, theta_zero);
title('Zero Position');

figure;
ax2 = axes;
drawSkeleton(ax2, theta_test);
title('Test Position [30, 45, -30, 0, 60, 0]');

%% TEST 5: Animate skeleton through a sweep of joint 1
disp('--- TEST 5: Animating joint 1 sweep ---');
figure;
ax3 = axes;
for t1 = 0:5:90
    drawSkeleton(ax3, [t1, 0, 0, 0, 0, 0]);
    title(ax3, sprintf('Joint 1 = %d deg', t1));
    pause(0.05);
end

%% TEST 6: Trajectory generation
disp('--- TEST 6: Trajectory points ---');
p_start = pos;   % use zero position EE as start
p_end   = [0.5; 0.3; 1.0];
points  = generateTrajectory(p_start, p_end, 10, 'Linear');
disp('Linear trajectory waypoints (10 steps):');
disp(points);

%% TEST 7: Inverse kinematics
disp('--- TEST 7: IK to reach [0.5, 0.3, 1.0] ---');
theta_ik = inverseKinematics([0.5; 0.3; 1.0], theta_zero);
fprintf('Solved joint angles: %.2f  %.2f  %.2f  %.2f  %.2f  %.2f\n', theta_ik);
[pos_check, ~] = getEEPose(theta_ik);
fprintf('Resulting EE pos:    X=%.4f  Y=%.4f  Z=%.4f\n', pos_check(1), pos_check(2), pos_check(3));
fprintf('Error from target:   %.5f m\n', norm(pos_check - [0.5;0.3;1.0]));

disp('--- All tests done ---');