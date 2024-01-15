function Param = SystemParamInitialization()
    Param.dof = 2;
    Param.m1 = 1; %kg
    Param.m2 = 1; %kg
    Param.l=1; %m
    Param.g = 9.81;
    Param.Kp = diag([50, 50]);
    Param.Kd = diag([25, 25]);
    Param.dt = 0.001;
    Param.totalTime = 3;
    Param.base_pos = [0;0];
end