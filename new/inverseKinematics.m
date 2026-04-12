function theta = inverseKinematics(target_pos, target_rpy, theta_init)
    theta = theta_init;
    max_iter = 1000;
    tol = 1e-3;
    alpha = 0.5;

    limits = [-160 160;
              -160 160;
              -160 160;
              -160 160;
              -160 160;
              -160 160];

    % Phase 1: position only
    for iter = 1:max_iter
        [pos, ~, ~] = getEEPose(theta);
        error = target_pos - pos;

        if norm(error) < tol
            break;
        end

        J = zeros(3, 6);
        delta = 0.01;
        for k = 1:6
            theta_plus = theta;
            theta_plus(k) = theta_plus(k) + delta;
            [pos_plus, ~, ~] = getEEPose(theta_plus);
            J(:,k) = (pos_plus - pos) / deg2rad(delta);
        end

        dtheta = alpha * pinv(J) * error;
        theta = theta + rad2deg(dtheta)';

        for k = 1:6
            theta(k) = max(limits(k,1), min(limits(k,2), theta(k)));
        end
    end

    [pos, ~, ~] = getEEPose(theta);
    phase1_err = norm(target_pos - pos);
    theta_phase1 = theta;  % save phase 1 result

    % Phase 2: position + orientation
    for iter = 1:max_iter
        [pos, ~, rpy] = getEEPose(theta);
        pos_error = target_pos - pos;
        rpy_error = target_rpy - rpy;
        rpy_error = mod(rpy_error + 180, 360) - 180;
        error = [pos_error; rpy_error * 0.01];

        if norm(error) < tol
            return;
        end

        J = zeros(6, 6);
        delta = 0.01;
        for k = 1:6
            theta_plus = theta;
            theta_plus(k) = theta_plus(k) + delta;
            [pos_plus, ~, rpy_plus] = getEEPose(theta_plus);
            J(:,k) = [pos_plus - pos; (rpy_plus - rpy) * 0.01] / deg2rad(delta);
        end

        dtheta = alpha * pinv(J) * error;
        theta = theta + rad2deg(dtheta)';

        for k = 1:6
            theta(k) = max(limits(k,1), min(limits(k,2), theta(k)));
        end
    end

    % If phase 2 made position worse, revert to phase 1
    [pos, ~, rpy] = getEEPose(theta);
    if norm(pos - target_pos) > phase1_err * 2
        theta = theta_phase1;
    end
end