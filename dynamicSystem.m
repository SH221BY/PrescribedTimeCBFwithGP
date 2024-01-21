function [q, q_dot] = dynamicSystem(tau, q, q_dot, SystemParam,dt)
    [M, C, G] = TwoLinkManipulatordynamicsGenerator(SystemParam, q, q_dot);
    q_ddot = inv(M) * (tau - C - G);
    q_dot = q_dot + q_ddot * dt;
    q = q + q_dot * dt;
end


