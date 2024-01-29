function [M, C, G] = TwoLinkRobotSimplified(SystemParam, q, q_dot)
    C1 = cos(q(1,1));
    C2 = cos(q(2,1));
    S2 = sin(q(2,1));
    C12 = cos(q(1,1)+q(2,1));
    M = [ SystemParam.dynSimpModel.m11Const + SystemParam.dynSimpModel.m11C2*C2, SystemParam.dynSimpModel.m12Const+SystemParam.dynSimpModel.m12C2*C2 ;
          SystemParam.dynSimpModel.m21Const + SystemParam.dynSimpModel.m21C2*C2,        SystemParam.dynSimpModel.m22Const               ];
    C = [ ( SystemParam.dynSimpModel.C11_S2_Qdot21_2*q_dot(2,1)^2 + SystemParam.dynSimpModel.C11_S2_Qdot11_Qdot21*q_dot(1,1)*q_dot(2,1) )*S2;
            SystemParam.dynSimpModel.C21_S2_Qdot11_2*S2*q_dot(1,1)^2];

    G = [ SystemParam.dynSimpModel.G11_C1*C1 + SystemParam.dynSimpModel.G11_C12*C12;
          SystemParam.dynSimpModel.G21_C12*C12];
end