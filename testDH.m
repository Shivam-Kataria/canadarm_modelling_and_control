function testDH()
    % Try swapping a and d to match CAD orientation
    combos = {
        [0,     0,     7.37,  0,     0,     0    ], [0,     0.722, 0.508, 0,     7.04,  0.731], [-90, 90, 0, -90, 90, -90];  % original
        [0,     0,     0.508, 0,     0,     0    ], [0,     0.722, 7.37,  0,     7.04,  0.731], [-90, 90, 0, -90, 90, -90];  % swap a3,d3
        [0,     0.722, 7.37,  0,     7.04,  0.731], [0,     0,     0.508, 0,     0,     0    ], [-90, 90, 0, -90, 90, -90];  % swap all a,d
        [0,     0,     7.37,  0,     0,     0    ], [0,     0.722, 0.508, 0,     7.04,  0.731], [90, -90, 0, 90, -90, 90];   % flip alphas
    };

    CAD_link5 = [14.799, 1.732, 1.246];
    fprintf('Target Frame5: X=%.3f Y=%.3f Z=%.3f\n\n', CAD_link5(1), CAD_link5(2), CAD_link5(3));

    for k = 1:size(combos,1)
        a     = combos{k,1};
        d     = combos{k,2};
        alpha = combos{k,3};

        T01 = DHlocal(a(1),d(1),alpha(1),0);
        T12 = DHlocal(a(2),d(2),alpha(2),0);
        T23 = DHlocal(a(3),d(3),alpha(3),0);
        T34 = DHlocal(a(4),d(4),alpha(4),0);
        T45 = DHlocal(a(5),d(5),alpha(5),0);
        T56 = DHlocal(a(6),d(6),alpha(6),0);
        T02=T01*T12; T03=T02*T23; T04=T03*T34; T05=T04*T45;

        p5 = T05(1:3,4);
        err = norm(p5 - CAD_link5');
        fprintf('Combo %d: Frame5=%.3f,%.3f,%.3f  error=%.3f\n', k, p5(1),p5(2),p5(3), err);
        fprintf('  a=[%.3f %.3f %.3f %.3f %.3f %.3f]\n', a);
        fprintf('  d=[%.3f %.3f %.3f %.3f %.3f %.3f]\n', d);
        fprintf('  alpha=[%d %d %d %d %d %d]\n\n', alpha);
    end
end

function T = DHlocal(a, d, alpha, theta)
    alpha = deg2rad(alpha);
    theta = deg2rad(theta);
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end