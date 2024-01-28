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
         function obj = PreCBFGP_2LinkManipulator(PreTime, DyParam, TrainModel)
            %obj.CBFProperty = struct('PrescribedTime',[],'MaxBlowupValue',[],'NumConstraint',[],'H_offset',[],'dHdxn',[],'CInitial',[]);
            obj.CBFProperty.PrescribedTime = PreTime;
            obj.CBFProperty.MaxBlowupValue = 6000;
            obj.CBFProperty.NewT0 = PreTime;
            obj.CBFProperty.NewTp = PreTime + 7;
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

         function [obj, u_safe, a_org_coe, a_uncertainty,a_gp_coe, b_coe, errorbound, h1, h2, h3] = ComputeSafeU(obj,u_norm,q,q_dot,t0,t,uncetaintyFlag)
            %% debug
            a_org_coe = zeros(obj.CBFProperty.NumConstraint,1);
            a_uncertainty=zeros(obj.CBFProperty.NumConstraint,1);
            a_gp_coe = zeros(obj.CBFProperty.NumConstraint,1);
            b_coe = zeros(4,2);
            errorbound = 0;
            u_safe = u_norm;
            %% main process
            if t > obj.CBFProperty.NewTp
                h1 = zeros(4,1);
                h2 = zeros(4,1);
                h3 = zeros(4,1);
                return
            end

            if t >= (obj.CBFProperty.PrescribedTime) && t <= obj.CBFProperty.NewTp %early return, nominal control input
                if obj.CBFProperty.SwithFlag == false
                    hi_dot = obj.CBFProperty.dHdxn*q_dot;
                    hi = obj.CBFProperty.H_offset*[q;1];
                    obj = obj.setCconstant(max(-hi_dot./hi,0)+2*ones(4,1));
                    obj.CBFProperty.SwithFlag = true;
                end
                mu=GetMuBlowupFunc(obj.CBFProperty.NewT0,t,obj.CBFProperty.NewTp ,2,obj.CBFProperty.MaxBlowupValue);
                mudot = GetMu2dot(obj.CBFProperty.NewT0,t,obj.CBFProperty.NewTp,obj.CBFProperty.MaxBlowupValue);
        
            elseif t < obj.CBFProperty.PrescribedTime %Prescribed time process
                
                %% parameter
                % alg setting
                mu=GetMuBlowupFunc(t0,t,obj.CBFProperty.PrescribedTime ,2,obj.CBFProperty.MaxBlowupValue);
                mudot = GetMu2dot(t0,t,obj.CBFProperty.PrescribedTime,obj.CBFProperty.MaxBlowupValue);
            end
            %% get dynamic parameter
            [M, C, G] = NominalTwoLinkManipulatordynamicsGenerator(obj.DySysParam, q, q_dot);
            %% coefficient
            A1 = obj.CBFProperty.H_offset(:,1:2) * q + obj.CBFProperty.H_offset(:,3);
            A2 = obj.CBFProperty.H_offset(:,1:2) * q_dot;
            invM = inv(M);
            c1 = obj.CBFProperty.CInitial(:,1);
            c2 = obj.CBFProperty.CInitial(:,2);
            a = obj.CBFProperty.H_offset(:,1:2) * invM * (-C-G) + (c1*mudot+c1.*c2*mu^2).*A1 + (c1*mu+c2*mu).*A2;
            b = obj.CBFProperty.H_offset(:,1:2) * invM;

            

            a_org_coe = a;
            b_coe = b;
            Uncertainty = GenerateUncertainty(q);
            if uncetaintyFlag == 1 % normal case, no uncertainty and GP
                %do noting 
            elseif (uncetaintyFlag == 2) % with uncertainty without GP
                a = a - obj.CBFProperty.H_offset(:,1:2) * Uncertainty;
                a_uncertainty = a;
                a_gp_coe = a_org_coe;
            elseif (uncetaintyFlag == 3) % with uncertainty and GP
                a = a - obj.CBFProperty.H_offset(:,1:2) *Uncertainty;
                
                a_uncertainty = a;
                
                [a, errorbound] = ConsiderGPerrorboundCBF(a, obj.GPModel, q, obj.CBFProperty.dHdxn);
                a_gp_coe = a;
                
            elseif (uncetaintyFlag == 4) % with uncertainty and uncertainty function
                a = a - obj.CBFProperty.H_offset(:,1:2) * Uncertainty;
                a_uncertainty = a;
                a = a + obj.CBFProperty.H_offset(:,1:2) * Uncertainty;
                a_gp_coe = a_org_coe;
            end

            % classical QP method
            % modify to only h1 for q1
            u_safe = optimizeControlInput(u_norm, a, b, t, obj.CBFProperty.PrescribedTime);

            % debug
            h1 = obj.CBFProperty.H_offset*[q;1];
            h1_dot = obj.CBFProperty.dHdxn*q_dot;
            h2 = h1_dot + c1*mu.*h1;
            h3 = obj.CBFProperty.dHdxn * (invM * (u_safe-C-G)) + c1*mudot.*h1 ...
                                        + c1*mu.*h1_dot + c2*mu.*h2;
            h3_uncertainty = a_org_coe+b*u_safe;
            h3_GP = a_gp_coe+b*u_safe;
            
            [mean,var,eta,beta,gamma,eta_min] = obj.GPModel.predict(q);
            du = Uncertainty - mean;
            hndu = obj.CBFProperty.dHdxn*du;
            phi=-abs(obj.CBFProperty.dHdxn)*[eta;eta];
         end
     end

end





