function [u_safe, M, C, G] = PresrcibedTime_CBF(u_norm,q,q_dot,t,PreCBFParam,SystemParam)
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
        a = inv(M)*(C+G)+PreCBFParam.c1*(-q+PreCBFParam.h1_offset).*mudot-PreCBFParam.c1*mu*q_dot+PreCBFParam.c2*mu*(-q_dot+PreCBFParam.c1*mu*(-q+PreCBFParam.h1_offset)) + GenerateUncertainty(q);
        b = -inv(M);
    
        a_paper = a(1);
        b_paper = [b(1,1);b(1,2)];
        bT_paper = transpose(b_paper);
        
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