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
             obj.CBFProperty.CInitial = c_padding*ones(obj.CBFProperty.NumConstraint,2); % c1, c2 
         end

         function [u_safe, M, C, G, a_org_coe, a_uncertainty,a_gp_coe, b_coe, errorbound] = ComputeSafeU(obj,u_norm,q,q_dot,t0,t,uncetaintyFlag)
            %% debug
            a_org_coe = zeros(obj.CBFProperty.NumConstraint,1);
            a_uncertainty=zeros(obj.CBFProperty.NumConstraint,1);
            a_gp_coe = zeros(obj.CBFProperty.NumConstraint,1);
            b_coe = zeros(4,2);
            errorbound = 0;
            %% parameter
            % alg setting
            mu=GetMuBlowupFunc(t0,t,obj.CBFProperty.PrescribedTime ,2,obj.CBFProperty.MaxBlowupValue);
            mudot = GetMu2dot(t0,t,obj.CBFProperty.PrescribedTime,obj.CBFProperty.MaxBlowupValue);
            %% get dynamic parameter
            [M, C, G] = NominalTwoLinkManipulatordynamicsGenerator(obj.DySysParam, q, q_dot);
            u_safe = u_norm;
            %% main process
            if t > (obj.CBFProperty.PrescribedTime) %early return, nominal control input
                return
            end
        
            if t <= obj.CBFProperty.PrescribedTime %Prescribed time process
                % coefficient
                A1 = obj.CBFProperty.H_offset(:,1:2) * q + obj.CBFProperty.H_offset(:,3);
                A2 = obj.CBFProperty.H_offset(:,1:2) * q_dot;
                invM = inv(M);
                c1 = obj.CBFProperty.CInitial(:,1);
                c2 = obj.CBFProperty.CInitial(:,2);
                a = obj.CBFProperty.H_offset(:,1:2) * invM * (-C-G) + (c1*mudot+c1.*c2*mu^2).*A1 + (c1*mu+c2*mu).*A2;
                b = obj.CBFProperty.H_offset(:,1:2) * invM;

                a_org_coe = a;
                b_coe = b;
                if uncetaintyFlag == 1 % normal case, no uncertainty and GP
                    %do noting 
                elseif (uncetaintyFlag == 2) % with uncertainty without GP
                    a = a - obj.CBFProperty.H_offset(:,1:2) * GenerateUncertainty(q);
                    a_uncertainty = a;
                    a_gp_coe = a_org_coe;
                elseif (uncetaintyFlag == 3) % with uncertainty and GP
                    a = a - obj.CBFProperty.H_offset(:,1:2) * GenerateUncertainty(q);
                    
                    a_uncertainty = a;
                    
                    [a, errorbound] = ConsiderGPerrorboundCBF(a, obj.GPModel, q, obj.CBFProperty.dHdxn);
                    a_gp_coe = a;
                    
                elseif (uncetaintyFlag == 4) % with uncertainty and uncertainty function
                    a = a - GenerateUncertainty(q); 
                    a_uncertainty = a;
                    a = a + GenerateUncertainty(q);
                    a_gp_coe = a_org_coe;
                end

                % classical QP method
                % modify to only h1 for q1
                u_safe = optimizeControlInput(u_norm, a, b);
        
            end
         end
     end

end





