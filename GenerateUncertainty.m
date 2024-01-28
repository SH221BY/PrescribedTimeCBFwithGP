function uncertainty = GenerateUncertainty(q)
    dof = size(q,1);
    q_inv = flip(q);
    %uncertainty = 5*sin(q) + 3*cos(q_inv) + GenerateGaussiaNoise(0,0.0001, dof);
    uncertainty = 50*sin(q) + 31*cos(q_inv) + GenerateGaussiaNoise(0,0.0001, dof);
end