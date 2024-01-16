function mod_CBF_coe = ConsiderGPerrorboundCBF(org_CBF_coe, GP_model, x_data, dhndxn)
    [mu,var,eta,beta,gamma,eta_min] = GP_model.predict(x_data);
    phi = - abs(dhndxn) *abs(eta);
    %phi = - dhndxn * (sqrt(beta)*var + gamma);   
    mod_CBF_coe = org_CBF_coe + phi + mu;
    %mod_CBF_coe = org_CBF_coe;
end