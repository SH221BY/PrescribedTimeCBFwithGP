function mu = GetMuBlowupFunc(t0,t,PrescribedTime,n,max_mu)
    dec = PrescribedTime+t0-t;
    if (dec < 0.00001 && dec >-0.00001)
        mu = max_mu;
    else
        mu=(PrescribedTime/dec)^n;
    end
    
    if(mu>max_mu)
        mu = max_mu;
    end
end