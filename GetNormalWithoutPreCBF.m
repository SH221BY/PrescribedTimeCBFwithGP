function tor_org = GetNormalWithoutPreCBF(TimeLen, q_desired, q, q_dot, SystemParam)
    for i = 1:TimeLen
        [u_org, error] = PDController(q_desired(:,i), q, q_dot, SystemParam);
    
        % input to real system
        [q, q_dot] = dynamicSystem(u_org, q, q_dot, SystemParam);
    
        % record result
        tor_org(:,i) = u_org;
    end
end