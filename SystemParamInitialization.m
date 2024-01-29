function Param = SystemParamInitialization(TrajFlag)
    Param.dof = 2;
    Param.m1 = 1; %kg
    Param.m2 = 1; %kg
    Param.l=1; %m
    Param.g = 9.81;
    Param.base_pos = [0;0];
    Param.dynSimpModel.m11Const = Param.m1*Param.l^2/3 + 4*Param.m2*Param.l^2/3;
    Param.dynSimpModel.m11C2 = Param.m2*Param.l^2;
    Param.dynSimpModel.m12Const = Param.m2*Param.l^2/3;
    Param.dynSimpModel.m12C2 = 0.5*Param.m2*Param.l^2;
    Param.dynSimpModel.m21Const =  Param.m2*Param.l^2/3;
    Param.dynSimpModel.m21C2 = 0.5*Param.m2*Param.l^2;
    Param.dynSimpModel.m22Const =  Param.m2*Param.l^2/3;

    Param.dynSimpModel.C11_S2_Qdot21_2 = -0.5*Param.m2*Param.l^2 ;
    Param.dynSimpModel.C11_S2_Qdot11_Qdot21 = - Param.m2*Param.l^2;
    Param.dynSimpModel.C21_S2_Qdot11_2 = 0.5*Param.m2*Param.l^2;

    Param.dynSimpModel.G11_C1 = 0.5*Param.m1*Param.g*Param.l + Param.m2*Param.g*Param.l;
    Param.dynSimpModel.G11_C12 = 0.5*Param.m2*Param.g*Param.l;
    Param.dynSimpModel.G21_C12 = 0.5*Param.m2*Param.g*Param.l;

    Param.Kp = diag([200, 200]);
    Param.Kd = diag([30, 30]);
    Param.Ki = diag([700, 700]);
    Param.Kv = diag([35, 35]);

    if TrajFlag == 2 || TrajFlag == 3
        Param.Kp = diag([50, 50]);
        Param.Kd = diag([0, 0]);
        Param.Ki = diag([0, 0]);
        Param.Kv = diag([25, 25]);
    end
end