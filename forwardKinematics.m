function [T01,T02,T03,T04,T05,T06] = forwardKinematics(theta)
    % DH in decimeters to match STL scale
    DH_a     = [0,     7.22,  73.7,  0,     70.4,  7.31 ];
    DH_d     = [0,     0,     5.08,  0,     0,     0    ];
    DH_alpha = [-90,   90,    0,     -90,   90,    -90  ];

    T01 = DHmat(DH_a(1),DH_d(1),DH_alpha(1),theta(1));
    T12 = DHmat(DH_a(2),DH_d(2),DH_alpha(2),theta(2));
    T23 = DHmat(DH_a(3),DH_d(3),DH_alpha(3),theta(3));
    T34 = DHmat(DH_a(4),DH_d(4),DH_alpha(4),theta(4));
    T45 = DHmat(DH_a(5),DH_d(5),DH_alpha(5),theta(5));
    T56 = DHmat(DH_a(6),DH_d(6),DH_alpha(6),theta(6));

    T02 = T01*T12; T03 = T02*T23;
    T04 = T03*T34; T05 = T04*T45; T06 = T05*T56;
end