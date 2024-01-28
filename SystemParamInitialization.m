function Param = SystemParamInitialization(TrajFlag)
    Param.dof = 2;
    Param.m1 = 1; %kg
    Param.m2 = 1; %kg
    Param.l=1; %m
    Param.g = 9.81;
    Param.base_pos = [0;0];
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