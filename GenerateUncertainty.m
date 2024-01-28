function uncertainty = GenerateUncertainty(q)
    dof = size(q,1);
    q_inv = flip(q);
    uncertainty = 50*sin(q) + 31*cos(q_inv) + [0;0]+GenerateGaussiaNoise(0,0.0001, dof);
end