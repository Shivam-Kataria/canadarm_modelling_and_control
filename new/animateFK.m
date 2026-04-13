function theta_final = animateFK(LinkPatches, V0_orig, theta_current, theta_target)
    n_frames = 50;
    for i = 1:n_frames
        alpha = i / n_frames;
        theta_interp = theta_current + alpha * (theta_target - theta_current);
        updateRobot(LinkPatches, V0_orig, theta_interp);
    end
    theta_final = theta_target;
end