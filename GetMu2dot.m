function mudot = GetMu2dot(t0,t,PrescribedTime,max_mu)
    dec = PrescribedTime+t0-t;
    if (dec < 0.00001 && dec >-0.00001)
        mudot = max_mu;
    else
        mudot=2*(PrescribedTime/dec)^3/PrescribedTime;
    end
    
    if(mudot>max_mu)
        mudot = max_mu;
    end
end