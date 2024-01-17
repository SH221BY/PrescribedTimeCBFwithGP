function [u_safe, M, C, G, a_org_coe, a_uncertainty,a_gp_coe, b_coe, errorbound] = PresrcibedTime_CBF(u_norm,q,q_dot,t,PreCBFParam,SystemParam,GPModel,uncetaintyFlag)
    a_org_coe = 0;
    a_uncertainty=0;
    a_gp_coe = 0;
    b_coe = [0;0];
    errorbound = 0;
    %% parameter
    % alg setting
    T_dash = PreCBFParam.PrescribedTime + PreCBFParam.SmoothTime;
    t0=0;
    Max_mu = 6000;
    mu=GetMuBlowupFunc(t0,t,PreCBFParam.PrescribedTime,2,Max_mu);
    mudot = GetMu2dot(t0,t,PreCBFParam.PrescribedTime,Max_mu);

    %get dynamic parameter
    [M, C, G] = NominalTwoLinkManipulatordynamicsGenerator(SystemParam, q, q_dot);

    u_safe = u_norm;
    if t > (T_dash)
        return
    end

    if t <= PreCBFParam.PrescribedTime 
        %% coefficient
        if uncetaintyFlag == 1 % normal case, no uncertainty and GP
            a = inv(M)*(C+G)+PreCBFParam.c1*(-q+PreCBFParam.h1_offset).*mudot-PreCBFParam.c1*mu*q_dot+PreCBFParam.c2*mu*(-q_dot+PreCBFParam.c1*mu*(-q+PreCBFParam.h1_offset));
            a_org_coe = a(1);
            a_gp_coe = a_org_coe;
        elseif (uncetaintyFlag == 2) % with uncertainty without GP
            a = inv(M)*(C+G)+PreCBFParam.c1*(-q+PreCBFParam.h1_offset).*mudot-PreCBFParam.c1*mu*q_dot+PreCBFParam.c2*mu*(-q_dot+PreCBFParam.c1*mu*(-q+PreCBFParam.h1_offset)) - GenerateUncertainty(q);
            a_org_coe = a(1);
            a_gp_coe = a_org_coe;
        elseif (uncetaintyFlag == 3) % with uncertainty and GP
            Uncertainty = GenerateUncertainty(q);
            a = inv(M)*(C+G)+PreCBFParam.c1*(-q+PreCBFParam.h1_offset).*mudot-PreCBFParam.c1*mu*q_dot+PreCBFParam.c2*mu*(-q_dot+PreCBFParam.c1*mu*(-q+PreCBFParam.h1_offset));
            a_org_coe = a(1);
            
            a = a-Uncertainty;
            a_uncertainty = a(1);
            
            dhndxn = -1;
            [a, errorbound] = ConsiderGPerrorboundCBF(a, GPModel, q, dhndxn);
            a_gp_coe = a(1);
            
        elseif (uncetaintyFlag == 4) % with uncertainty and uncertainty function
            a = inv(M)*(C+G)+PreCBFParam.c1*(-q+PreCBFParam.h1_offset).*mudot-PreCBFParam.c1*mu*q_dot+PreCBFParam.c2*mu*(-q_dot+PreCBFParam.c1*mu*(-q+PreCBFParam.h1_offset)) - GenerateUncertainty(q) + GenerateUncertainty(q);
            a_org_coe = a(1);
            a_gp_coe = a_org_coe;
        end
        
        
        b = -inv(M);
    
        a_paper = a(1);
        b_paper = [b(1,1);b(1,2)];
        bT_paper = transpose(b_paper);

        b_coe = b_paper;
        
        % KKT method
        det = a_paper+bT_paper*u_norm;
        if(det)>=0
            u_safe=u_norm;
        else
            bnorm_square = b_paper(1)^2+b_paper(2)^2;
            u_safe = u_norm-(det/bnorm_square) * b_paper;
        end
    
        % classical QP method
        % modify to only h1 for q1
        %a(2,1) = 0;
        %b(2,1) = 0;
        %b(2,2) = 0;
        %u_safe = optimizeControlInput(u_norm, a, b);

    else %PrescribedTime ~ T_dash
       %mu1 = GetMuBlowupFunc(t0,t-PreCBFParam.PrescribedTime,PreCBFParam.SmoothTime,1,Max_mu);
       %phi=exp(-mu1+1);
       %u_safe = PreCBFParam.u_terminal * phi + (1-phi)*u_norm;
    end
end