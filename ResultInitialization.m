function result = ResultInitialization(dof, TimeLen)
    result.q_r = zeros(dof,TimeLen);
    result.qdot_r = zeros(dof,TimeLen);
    result.tor_org = zeros(dof,TimeLen);
    result.tor_r = zeros(dof,TimeLen);
    result.torsafe_r = zeros(dof,TimeLen);
    result.q_error_r = zeros(dof,TimeLen);
    result.end_effector_pos_r = zeros(dof,TimeLen);
    result.end_effector_pos_cmd = zeros(dof,TimeLen);
    result.a_coe_org_paper = zeros(1,TimeLen);
    result.a_coe_withUncertainty = zeros(1,TimeLen);
    result.a_coe_withGP_paper = zeros(1,TimeLen);
    result.b_coe_paper = zeros(dof,TimeLen);
    result.CBFCon_with_uncertainty = zeros(1,TimeLen);
    result.CBFCon_with_GP = zeros(1,TimeLen);
    result.errorbound = zeros(1,TimeLen);
end