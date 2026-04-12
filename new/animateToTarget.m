function theta_final = animateToTarget(LinkPatches, V0_orig, theta_current, target_pos, target_rpy)
    % Solve IK
    theta_target = inverseKinematics(target_pos, target_rpy, theta_current);
    
    % Interpolate between current and target
    n_frames = 50;
    for i = 1:n_frames
        alpha = i / n_frames;
        theta_interp = theta_current + alpha * (theta_target - theta_current);
        updateRobot(LinkPatches, V0_orig, theta_interp);
    end
    
    theta_final = theta_target;
end