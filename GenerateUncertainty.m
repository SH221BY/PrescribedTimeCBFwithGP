function uncertainty = GenerateUncertainty(q)
    dof = size(q,1);
    q_inv = flip(q);
    uncertainty = -2*sin(q) - 3*cos(q_inv) + GenerateGaussiaNoise(0,0.01, dof);
end