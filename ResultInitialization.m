function result = ResultInitialization(dof, TimeLen)
    result.q_r = zeros(dof,TimeLen);
    result.qdot_r = zeros(dof,TimeLen);
    result.tor_org = zeros(dof,TimeLen);
    result.tor_r = zeros(dof,TimeLen);
    result.torsafe_r = zeros(dof,TimeLen);
    result.q_error_r = zeros(dof,TimeLen);
    result.end_effector_pos_r = zeros(dof,TimeLen);
    result.end_effector_pos_cmd = zeros(dof,TimeLen);
end