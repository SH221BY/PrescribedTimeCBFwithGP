function [q_desired, t, q_dot_desired] = jointTrajectoryGenerator(totalTime, dt)
    t = 0:dt:totalTime;
    q_desired = [sin(t); cos(t)];
    q_dot_desired = [cos(t); -sin(t)] / dt;

    %q_desired = [cos(t); sin(t)];
    %q_dot_desired = [-sin(t); cos(t)] / dt;
end