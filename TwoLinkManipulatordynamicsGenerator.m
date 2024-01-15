function [M, C, G] = TwoLinkManipulatordynamicsGenerator(SystemParam, q, q_dot)
    C1 = cos(q(1,1));
    C2 = cos(q(2,1));
    S2 = sin(q(2,1));
    C12 = cos(q(1,1)+q(2,1));
    M = [ SystemParam.m1*SystemParam.l^2/3 + 4*SystemParam.m2*SystemParam.l^2/3+SystemParam.m2*C2*SystemParam.l^2, SystemParam.m2*SystemParam.l^2/3+0.5*SystemParam.m2*SystemParam.l^2*C2 ;
          SystemParam.m2*SystemParam.l^2/3 + 0.5*SystemParam.m2*SystemParam.l^2*C2,        SystemParam.m2*SystemParam.l^2/3                 ];

    C = [ -0.5*SystemParam.m2*S2*SystemParam.l^2*q_dot(2,1)^2 - SystemParam.m2*S2*SystemParam.l^2*q_dot(1,1)*q_dot(2,1);
          0.5*SystemParam.m2*S2*SystemParam.l^2*q_dot(1,1)^2                                    ];

    G = [ 0.5*SystemParam.m1*SystemParam.g*SystemParam.l*C1 + 0.5*SystemParam.m2*SystemParam.g*SystemParam.l*C12 + SystemParam.m2*SystemParam.g*SystemParam.l*C1;
          0.5*SystemParam.m2*SystemParam.g*SystemParam.l*C12];

end