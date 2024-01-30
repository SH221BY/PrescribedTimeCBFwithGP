function uncertainty = GenerateUncertainty(q)
    dof = size(q,1);
    q_inv = flip(q);
    % uncertainty = 5*sin(q) + 3*cos(q_inv) + [0;35]+GenerateGaussiaNoise(0,0.01, dof);
    uncertainty = 5*sin(q) +3*cos(q_inv) + [0;30]+GenerateGaussiaNoise(0,0.001, dof); %org
    
end