function uncertainty = GenerateUncertainty(q,q_dot)
    dof = size(q,1);
    q_inv = flip(q);
    % uncertainty = 5*sin(q) + 3*cos(q_inv) + [0;35]+GenerateGaussiaNoise(0,0.01, dof);
    % uncertainty = 5*sin(q) +3*cos(q_inv) + [0;30]+GenerateGaussiaNoise(0,0.0001, dof); %org
    uncertainty = 5*sin(q) +3*cos(q_inv) + 0.00001*q_dot + [0;0]+GenerateGaussiaNoise(0,0.0001, dof); %org
end