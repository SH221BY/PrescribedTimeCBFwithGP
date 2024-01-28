function u_opt = optimizeControlInput(u_norm, a, B, t,Tp)
    % Ensure B is square and its dimensions match those of a and u_norm
    %if ~isequal(size(B,1), size(B,2), length(u_norm))
    %    error('Matrix B must be square and match the dimension of a and u_norm');
    %end

    % Number of control inputs
    n = length(u_norm);

    % Quadratic cost function parameters
    H = 2 * eye(n);  % Quadratic term (since norm(u-u_norm)^2)
    f = -2 * u_norm; % Linear term

    % Constraints (a + B*u >= 0)
    A = -B;          % Inequality constraints matrix
    b_constr = a;    % Inequality constraints vector

    % Bounds for control input (if any, otherwise set to [])
    lb = []; % Lower bound
    ub = []; % Upper bound

    % Solving the QP problem, Ax<=b
    
    options = optimoptions('quadprog', 'Display', 'off');

    %if(Tp-t) <= 0.005
    %    t
    %    options = optimoptions('quadprog');
    %end
    %options = optimoptions('quadprog','ConstraintTolerance', 1e-6, 'OptimalityTolerance', 1e-6);
    u_opt = quadprog(H, f, A, b_constr, [], [], lb, ub, [], options);
end
