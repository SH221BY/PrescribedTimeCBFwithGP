function [mod_CBF_coe, eta] = ConsiderGPerrorboundCBF(org_CBF_coe, GP_model, x_data, dhndxn)
    [mu,var,eta,beta,gamma,eta_min] = GP_model.predict(x_data);
    phi = - abs(dhndxn) * abs(eta);
    mod_CBF_coe = org_CBF_coe + phi + mu;
end