function [tau, error] = PDController(q_desired, q_current, q_dot_current, SystemParam)
    error = q_desired - q_current;
    error_dot = SystemParam.Kp*error -q_dot_current;
    tau = SystemParam.Kp * error + SystemParam.Kd * error_dot;
end
