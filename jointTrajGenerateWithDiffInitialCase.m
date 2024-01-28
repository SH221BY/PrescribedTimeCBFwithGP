function [q_desired, t, q_dot_desired] = jointTrajGenerateWithDiffInitialCase(totalTime, dt,Trajflag)
    t = 0:dt:totalTime;
    % Safe to safe
    if Trajflag == 1
        Scalar = pi/6;
        lag_rotation = pi/2;
        centerPt = [3*pi/8;0];
  
    % unsafe to safe
    elseif Trajflag == 2
        Scalar = 4*pi/4;
        lag_rotation = (2*pi)/4;
        centerPt = [4*pi/8;0*pi/8];
    % unsafe -> safe -> safe
    elseif Trajflag == 3
        Scalar = 4*pi/4;
        lag_rotation = (2*pi)/4;
        centerPt = [4*pi/8;0*pi/8];
    end
    q_desired = Scalar* [cos(t-lag_rotation); sin(t-lag_rotation)] + centerPt;
    q_dot_desired = Scalar * [-sin(t-lag_rotation); cos(t-lag_rotation)] / dt;
end