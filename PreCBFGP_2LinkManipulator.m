classdef PreCBFGP_2LinkManipulator
     properties
        DySysParam       % dynamic system parameter, struct
        GPModel          % model afgter training
        CBFProperty      % Property of CBF
                            % PrescribedTime   % Prescibed time, scalar
                            % MaxBlowupValue   % max value of blow  up function
                            % C_Initial        % c1, c2
                            % H_offset, dim x 3(gamma1)
                            % NumConstraint, dim, scalar
     end

     methods
         % ctor
         function obj = PreCBFGP_2LinkManipulator(PreTime, NextPrescribedTime,DyParam, TrainModel)
            %obj.CBFProperty = struct('PrescribedTime',[],'MaxBlowupValue',[],'NumConstraint',[],'H_offset',[],'dHdxn',[],'CInitial',[]);
            obj.CBFProperty.PrescribedTime = PreTime;
            obj.CBFProperty.MaxBlowupValue = 6000;
            obj.CBFProperty.NewT0 = PreTime;
            obj.CBFProperty.NewTp = PreTime + NextPrescribedTime;
            obj.DySysParam = DyParam;
            obj.GPModel = TrainModel; 
            obj = obj.SetCBFConstraint();
            obj = obj.SetInitialConstant();
         end
         
         function obj = SetCBFConstraint(obj)
             obj.CBFProperty.NumConstraint = 4;
             obj.CBFProperty.H_offset = zeros(obj.CBFProperty.NumConstraint,3); %gamma1*q1 + gamma2*q2 + gamma3 > 0
             obj.CBFProperty.H_offset = [  1,  0, 0;
                                          -1,  0, pi/2;
                                           1,  1, 0;
                                          -1, -1, pi/2 ];

             %gradient of h
             obj.CBFProperty.dHdxn = [ 1,  0;
                                      -1,  0;
                                       1,  1;
                                      -1, -1 ];


             
         end

         function obj = SetInitialConstant(obj)
             c_padding = 0.2;
             c_dot_padding = 0.2;
             obj.CBFProperty.CInitial = [ones(4,1)*c_padding, ones(4,1)*c_dot_padding]; % c1, c2
             obj.CBFProperty.SwithFlag = false;
         end

         function obj = setCconstant( obj, c_constant )
             obj.CBFProperty.CInitial(:,1) = c_constant;
         end

         function [obj, u_safe] = ComputeSafeU(obj,u_norm,q,q_dot,t0,t,uncetaintyFlag,M,C,G)
            %% debug
            u_safe = u_norm;
            %% main process
            if t > obj.CBFProperty.NewTp
                return
            end

            if t >= (obj.CBFProperty.PrescribedTime) && t <= obj.CBFProperty.NewTp %compute second section mu, mu_dot
                if obj.CBFProperty.SwithFlag == false
                    hi_dot = obj.CBFProperty.dHdxn*q_dot;
                    hi = obj.CBFProperty.H_offset*[q;1];
                    padding = 0.2;
                    obj = obj.setCconstant(max(-hi_dot./hi,0)+padding*ones(4,1));
                    obj.CBFProperty.SwithFlag = true;
                end
                mu=GetMuBlowupFunc(obj.CBFProperty.NewT0,t,obj.CBFProperty.NewTp ,2,obj.CBFProperty.MaxBlowupValue);
                mudot = GetMu2dot(obj.CBFProperty.NewT0,t,obj.CBFProperty.NewTp,obj.CBFProperty.MaxBlowupValue);
        
            elseif t < obj.CBFProperty.PrescribedTime %compute first section mu, mu_dot
                mu=GetMuBlowupFunc(t0,t,obj.CBFProperty.PrescribedTime ,2,obj.CBFProperty.MaxBlowupValue);
                mudot = GetMu2dot(t0,t,obj.CBFProperty.PrescribedTime,obj.CBFProperty.MaxBlowupValue);
            end
            %% get dynamic parameter
            %% coefficient
            A1 = obj.CBFProperty.H_offset*[q;1];
            A2 = obj.CBFProperty.dHdxn * q_dot;
            invM = inv(M);
            c1 = obj.CBFProperty.CInitial(:,1);
            c2 = obj.CBFProperty.CInitial(:,2);
            a = obj.CBFProperty.H_offset(:,1:2) * invM * (-C-G) + (mudot+c2*mu^2).*c1.*A1 + (c1+c2)*mu.*A2;
            b = obj.CBFProperty.H_offset(:,1:2) * invM;

            Uncertainty = GenerateUncertainty(q);
            if uncetaintyFlag == 1 % normal case, no uncertainty and GP
                %do noting 
            elseif (uncetaintyFlag == 2) % with uncertainty without GP
                a = a - obj.CBFProperty.H_offset(:,1:2) * Uncertainty;
            elseif (uncetaintyFlag == 3) % with uncertainty and GP
                a = a - obj.CBFProperty.H_offset(:,1:2) * Uncertainty;
                [a, errorbound] = ConsiderGPerrorboundCBF(a, obj.GPModel, q, obj.CBFProperty.dHdxn);
            elseif (uncetaintyFlag == 4) % with uncertainty and uncertainty function
                a = a - obj.CBFProperty.H_offset(:,1:2) * Uncertainty;
                a = a + obj.CBFProperty.H_offset(:,1:2) * Uncertainty;
            end

            % classical QP method
            u_safe = optimizeControlInput(u_norm, a, b, t, obj.CBFProperty.PrescribedTime);
           
         end
     end

end





