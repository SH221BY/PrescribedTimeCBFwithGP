function PreCBFParam = PreCBFParamInitialization()
    PreCBFParam.PrescribedTime = 2; %sec
    PreCBFParam.SmoothTime = 0.5; %sec
    PreCBFParam.u_terminal = 0;
    
    %h1 -> -q1+0.6>0
    PreCBFParam.h1_offset = [10;0];
    
    % c
    c_padding = 0.2;
    h1_t0=-PreCBFParam.h1_offset(1);
    h1dot_t0=0;
    PreCBFParam.c1 = -h1dot_t0/h1_t0 + c_padding;
   
    h2_t0=PreCBFParam.c1*h1_t0;
    h2dot_t0=0;
    PreCBFParam.c2 = -h2dot_t0/h2_t0 + c_padding;

end