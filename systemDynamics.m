function x_dot = systemDynamics(t,x)
    global gSP error_l dt_s PreCBFGP t0_s UncertaintyFlag_s

    [M, C, G] = TwoLinkManipulatordynamicsGenerator(gSP, x(1:2), x(3:4));

    [u_norm, error] = PDController(GetCurDesire(t,x(1:2)), x(1:2), x(3:4), error_l, dt_s, gSP);
    % update error
    error_l = error;

    % Prescribed time CBF
    [PreCBFGP, u_safe ] = ...
        PreCBFGP.ComputeSafeU(u_norm,x(1:2),x(3:4),t0_s,t,UncertaintyFlag_s);

    % Compute the acceleration
    acceleration = M \ (-C - G + u_safe); % M^(-1)*(-C-G+u)

    % Construct the derivative of the state vector
    x_dot = [x(3:4); acceleration];
end