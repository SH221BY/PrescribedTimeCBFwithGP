function uncertainty = GenerateUncertainty(q)
    dof = size(q,1);
    q_inv = flip(q);
    %q_dot_inv = flip(q_dot);
    uncertainty = -2*sin(q) - 3*cos(q_inv) + GenerateGaussiaNoise(0,0.01, dof);
    %uncertainty = 2*sin(q) + cos(q_dot.^2) + 3*cos(q_inv.^3) + sin(q_dot_inv) + GenerateGaussiaNoise(0,0.0016, dof);
end