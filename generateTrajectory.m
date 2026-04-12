function points = generateTrajectory(p_start, p_end, N, type)
    points = zeros(N,3);

    if strcmp(type,'Linear')
        for i = 1:N
            s = i/N;
            points(i,:) = (p_start + s*(p_end - p_start))';
        end

    elseif strcmp(type,'Circular')
        center = (p_start + p_end) / 2;
        r      = norm(p_end(1:2) - p_start(1:2)) / 2;
        for i = 1:N
            angle       = 2*pi * i/N;
            points(i,1) = center(1) + r*cos(angle);
            points(i,2) = center(2) + r*sin(angle);
            points(i,3) = p_start(3) + (i/N)*(p_end(3)-p_start(3));
        end
    end
end