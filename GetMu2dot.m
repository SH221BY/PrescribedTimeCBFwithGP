function mudot = GetMu2dot(t0,t,PrescribedTime,max_mu)
    dec = PrescribedTime+t0-t;
    if (dec < 0.000001 && dec >-0.000001)
        mudot = max_mu;
    else
        mudot=2*(PrescribedTime/dec)^3/PrescribedTime+800*(t-t0);
    end
    
    if(mudot>max_mu)
        mudot = max_mu;
    end
    if(-mudot>max_mu)
        mudot = -max_mu;
    end
end