function Q_Desire =  GetCurDesire(t,x)
    global Trajflag_s

    if Trajflag_s == 1
        lag_rotation = pi/2;
        centerPt = [3*pi/8;0];
        Scalar = pi/6;
        
    % unsafe to safe
    elseif Trajflag_s == 2
        Scalar = 4*pi/4;
        lag_rotation = (2*pi)/4;
        centerPt = [4*pi/8;0*pi/8];
    elseif Trajflag_s == 3
        Scalar = 4*pi/4;
        lag_rotation = (-1*pi)/4;
        centerPt = [4*pi/8;0*pi/8];
    end
    Q_Desire = Scalar* [cos(t-lag_rotation); sin(t-lag_rotation)] + centerPt;
end