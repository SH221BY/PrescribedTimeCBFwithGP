function mu = GetMuBlowupFunc(t0,t,PrescribedTime,n,max_mu)
    dec = PrescribedTime+t0-t;
    if (dec < 0.000001 && dec >-0.000001)
        mu = max_mu;
    else
        mu=(PrescribedTime/dec)^n+400*(t-t0)^2;
    end
    
    if(mu>max_mu)
        mu = max_mu;
    end
end