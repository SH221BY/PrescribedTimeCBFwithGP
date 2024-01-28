function [tau, error] = PDController(q_desired, q_current, q_dot, Lasterror, dt, SystemParam)
    error = q_desired - q_current;
    Pvalue = SystemParam.Kp * error;
    DValue = SystemParam.Kd*(error - Lasterror)*dt;
    IValue = SystemParam.Ki*(Lasterror + error)*dt/2;
    error_dot = (Pvalue - q_dot);
    tau = Pvalue + DValue + IValue + SystemParam.Kv*error_dot;
end
