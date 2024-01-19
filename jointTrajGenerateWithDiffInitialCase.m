function [q_desired, t, q_dot_desired] = jointTrajGenerateWithDiffInitialCase(totalTime, dt,Trajflag)
    t = 0:dt:totalTime;
    q_desired = zeros(2,length(t));
    q_dot_desired = zeros(2,length(t));

    % Safe to safe
    if Trajflag == 1
        Scalar = 1;
        lag_rotation = pi/4;
        centerPt = [0.4;0];
        q_desired = Scalar* [cos(t-lag_rotation); sin(t-lag_rotation)] + centerPt;
        q_dot_desired = Scalar * [-sin(t-lag_rotation); cos(t-lag_rotation)] / dt;
    % unsafe to safe
    elseif Trajflag == 2
        Scalar = 1;
        lag_rotation = pi;
        q_desired = Scalar * [cos(t-lag_rotation); sin(t-lag_rotation)];
        q_dot_desired = Scalar * [-sin(t-lag_rotation); cos(t-lag_rotation)] / dt;
    end
end