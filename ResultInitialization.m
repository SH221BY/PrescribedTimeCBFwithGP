function result = ResultInitialization(dof, TimeLen)
    result.q_r = zeros(dof,TimeLen);
    result.qdot_r = zeros(dof,TimeLen);
    result.tor_org = zeros(dof,TimeLen);
    result.tor_r = zeros(dof,TimeLen);
    result.torsafe_r = zeros(dof,TimeLen);
    result.q_error_r = zeros(dof,TimeLen);
    result.end_effector_pos_r = zeros(dof,TimeLen);
    result.end_effector_pos_cmd = zeros(dof,TimeLen);
    % 1x1, just for test
    result.a_coe_org_paper_1 = zeros(1,TimeLen);
    result.a_coe_withUncertainty_1 = zeros(1,TimeLen);
    result.a_coe_withGP_paper_1 = zeros(1,TimeLen);
    result.b_coe_paper_1 = zeros(dof,TimeLen);
    result.CBFCon_with_uncertainty_1 = zeros(1,TimeLen);
    result.CBFCon_with_GP_1 = zeros(1,TimeLen);
    result.errorbound = zeros(1,TimeLen);
    % 4x1
    result.a_coe_org_paper = zeros(4,TimeLen);
    result.a_coe_withUncertainty = zeros(4,TimeLen);
    result.a_coe_withGP_paper = zeros(4,TimeLen);
    result.b_coe_paper = zeros(4,dof,TimeLen);
    result.CBFCon_with_uncertainty = zeros(4,TimeLen);
    result.CBFCon_with_GP = zeros(4,TimeLen);
    result.h1 = zeros(4,TimeLen);
    result.h2 = zeros(4,TimeLen);
    result.h3 = zeros(4,TimeLen);
end