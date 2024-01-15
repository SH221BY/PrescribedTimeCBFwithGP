function uncertainty = GenerateUncertainty(q,q_dot)
    dof = length(q);
    uncertainty = 2*sin(q) + cos(q_dot.^2) + GenerateGaussiaNoise(0,0.0016, dof);
end