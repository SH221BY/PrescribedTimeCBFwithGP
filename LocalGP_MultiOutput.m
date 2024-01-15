classdef LocalGP_MultiOutput < handle
	properties
		ActivateState = false;
		% Data
		X; % dim * DataNum
		Y; % DataNum * dim
		x_dim;
		y_dim;
		MaxDataQuantity = 100;
		DataQuantity = 0;
		xMax;
		xMin;
		% Parameter
		SigmaN; % with I matrix
		SigmaF; % kernel hyper param
		SigmaL; % kernel hyper param, the relationship level between different diemension
		% GP coefficient
		K;
		L;
		alpha;
		aux_alpha;
		% Error Bound
		tau = 1e-6;
		Lk;
		M;
		Lmu_set
		Lmu;
		omega_SigmaN_tau;
		Lsigma;
		delta;
		Lf_set;
		Lf;
	end
	methods
		function obj = LocalGP_MultiOutput(x_dim,y_dim,MaxDataQuantity, ...
				SigmaN,SigmaF,SigmaL)
			obj.MaxDataQuantity = MaxDataQuantity;
			obj.x_dim = x_dim;
			obj.y_dim = y_dim;
			obj.X = nan(x_dim,obj.MaxDataQuantity);
			obj.Y = nan(obj.MaxDataQuantity,y_dim);
% 			obj.xMax = -inf * ones(x_dim,1);
% 			obj.xMin =  inf * ones(x_dim,1);

			obj.K = nan(obj.MaxDataQuantity,obj.MaxDataQuantity);
			obj.L = nan(obj.MaxDataQuantity,obj.MaxDataQuantity);
			obj.aux_alpha = nan(obj.MaxDataQuantity,y_dim);
			obj.alpha = nan(obj.MaxDataQuantity,y_dim);

			obj.SigmaN = SigmaN;
			obj.SigmaF = SigmaF;
			obj.SigmaL = SigmaL;

			obj.Lmu = nan(y_dim,1);
			obj.omega_SigmaN_tau = nan;
			obj.Lk = norm(obj.SigmaF ^ 2 * exp(-0.5) ./ obj.SigmaL);
% 			obj.Lk = norm(1 ^ 2 * exp(-0.5) ./ 1);
			obj.delta = 0.05;
			obj.Lf_set = ones(y_dim,1);
			obj.Lf = sqrt(sum(obj.Lf_set .^ 2));
			obj.Lsigma = obj.SigmaF * norm(1 ./ obj.SigmaL);

			obj.ActivateState = true;
		end
		%% Reset
		function resetGP(obj)
			obj.X = nan(obj.x_dim,obj.MaxDataQuantity);
			obj.Y = nan(obj.MaxDataQuantity,obj.y_dim);
			obj.DataQuantity = 0;

			obj.K = nan(obj.MaxDataQuantity,obj.MaxDataQuantity);
			obj.L = nan(obj.MaxDataQuantity,obj.MaxDataQuantity);
			obj.aux_alpha = nan(obj.MaxDataQuantity,obj.y_dim);
			obj.alpha = nan(obj.MaxDataQuantity,obj.y_dim);

			obj.ActivateState = false;
		end
		%% Kernel Function, ARD Squared Exponential Kernel
		function kern = kernel(obj, Xi, Xj)%squared exponential kernel, K = SigmaF^2*exp( -1/(2SigmaL^2) * (xi-xj)T(xi-xj) )
			dx = size(Xi,1);
			if nargin == 2
				n = size(Xi,2);
				d = zeros(dx,1,n);
			elseif nargin == 3
				m = size(Xj,2);
				d = Xi - reshape(Xj,dx,1,m);
			end
			kern = permute((obj.SigmaF^2)*exp(-0.5*sum((d.^2)./(obj.SigmaL.^2),1)),[2 3 1]);
		end
		%% Add Point
		function flag = addPoint(obj, x, y)
			pts_local = obj.DataQuantity;
			if pts_local >= obj.MaxDataQuantity
				flag = -1;
			else
				obj.X(:,pts_local+1) = x;
				obj.Y(pts_local+1,:) = y';
				obj.DataQuantity = pts_local + 1;
				obj.updateParam;

% 				obj.xMax = max(obj.xMax,x);
% 				obj.xMin = min(obj.xMin,x);
				flag = 1;
			end
		end
		%% Update Parameter
		function updateParam(obj)
			LocalGP_DataQuantity = obj.DataQuantity;
			X_set = obj.X(:,1:LocalGP_DataQuantity);
			if obj.DataQuantity == 1 %first point in model
				new_K = obj.kernel(X_set, X_set) + obj.SigmaN .^ 2;
				new_L = chol(new_K,'lower');

				obj.K(1:LocalGP_DataQuantity,1:LocalGP_DataQuantity) = new_K;
				obj.L(1:LocalGP_DataQuantity,1:LocalGP_DataQuantity) = new_L;

% 				obj.M = 1;
				obj.omega_SigmaN_tau = 2 * obj.tau * obj.Lk * ...
					(1 + LocalGP_DataQuantity * norm(inv(new_K)) * obj.kernel(X_set));
				for i = 1:obj.y_dim
					y_set = obj.Y(1:LocalGP_DataQuantity,i);
					new_aux_alpha = new_L \ y_set;
					new_alpha = new_L' \ new_aux_alpha;

					obj.aux_alpha(1:LocalGP_DataQuantity,i) = new_aux_alpha;
					obj.alpha(1:LocalGP_DataQuantity,i) = new_alpha;
				end
			else
				old_X = X_set(:,1:end-1);
				new_x = X_set(:,end);

				old_K = obj.K(1:(LocalGP_DataQuantity - 1),1:(LocalGP_DataQuantity - 1));
				old_L = obj.L(1:(LocalGP_DataQuantity - 1),1:(LocalGP_DataQuantity - 1));
				

				b = obj.kernel(old_X,new_x);
				c = obj.kernel(new_x,new_x) + obj.SigmaN ^ 2;
				L1 = old_L \ b;
				L2 = sqrt(c - norm(L1)^2);
				new_K = [old_K,b;b',c];
				new_L = [...
					old_L,		zeros(LocalGP_DataQuantity-1,1);
					L1',		L2];

				obj.K(1:LocalGP_DataQuantity,1:LocalGP_DataQuantity) = new_K;
				obj.L(1:LocalGP_DataQuantity,1:LocalGP_DataQuantity) = new_L;

				for i = 1:obj.y_dim
					old_aux_alpha = obj.aux_alpha(1:(LocalGP_DataQuantity - 1),i);
					new_y = obj.Y(LocalGP_DataQuantity,i);
					new_aux_alpha = (new_y - L1' * old_aux_alpha) / L2;
					obj.aux_alpha(LocalGP_DataQuantity,i) = new_aux_alpha;

					new_alpha = new_L' \ obj.aux_alpha(1:LocalGP_DataQuantity,i);
					obj.alpha(1:LocalGP_DataQuantity,i) = new_alpha;
				end
			end
		end
		%% downdate parameter
		function downdateParam(obj,DeleteNr)
			LocalGP_DataQuantity = obj.DataQuantity;
			if LocalGP_DataQuantity == 0
				return;
			elseif LocalGP_DataQuantity > 1
				old_K = obj.K(1:LocalGP_DataQuantity,1:LocalGP_DataQuantity);
				old_L = obj.L(1:LocalGP_DataQuantity,1:LocalGP_DataQuantity);


				Lb = old_L((DeleteNr+1):end,(DeleteNr+1):end);
				lbc = old_L((DeleteNr+1):end,DeleteNr);
				Lb_star = cholupdate(Lb',lbc);
				Lb_star = Lb_star';

				obj.K(1:(LocalGP_DataQuantity-1),1:(LocalGP_DataQuantity-1)) = [...
					old_K(1:(DeleteNr-1),1:(DeleteNr-1)),	old_K(1:(DeleteNr-1),(DeleteNr+1):end);
					old_K((DeleteNr+1):end,1:(DeleteNr-1)),	old_K((DeleteNr+1):end,(DeleteNr+1):end)];
				obj.L(1:(LocalGP_DataQuantity-1),1:(LocalGP_DataQuantity-1)) = [...
					old_L(1:(DeleteNr-1),1:(DeleteNr-1)),	zeros(DeleteNr-1,LocalGP_DataQuantity-DeleteNr);
					old_L((DeleteNr+1):end,1:(DeleteNr-1)),	Lb_star];

				for i = 1:obj.y_dim
					old_aux_alpha = obj.aux_alpha(1:LocalGP_DataQuantity,i);

					aa = old_aux_alpha(1:(DeleteNr-1));
					ab = old_aux_alpha((DeleteNr+1):end);
					ac = old_aux_alpha(DeleteNr);

					obj.aux_alpha(1:(LocalGP_DataQuantity-1),i) = [...
						aa;	Lb_star \ (lbc * ac + Lb * ab)];
					obj.alpha(1:(LocalGP_DataQuantity-1),i) = ...
						obj.L(1:(LocalGP_DataQuantity-1),1:(LocalGP_DataQuantity-1)) \ ...
						obj.aux_alpha(1:(LocalGP_DataQuantity-1),i);
				end

			end % else LocalGP_DataQuantity == 1
			obj.DataQuantity = obj.DataQuantity - 1;
			obj.X(:,DeleteNr:obj.DataQuantity) = obj.X(:,(DeleteNr+1):(obj.DataQuantity+1));
			obj.Y(DeleteNr:obj.DataQuantity,:) = obj.Y((DeleteNr+1):(obj.DataQuantity+1),:);
		end
		%% Insert All Data, put data into member data and update kernel matrix
		function add_Alldata(obj,X_in,Y_in)
			if size(X_in,2) == obj.x_dim
				X_in = X_in';
			end
			if size(Y_in,1) == obj.y_dim
				Y_in = Y_in';
			end
			if size(X_in,2) ~= size(Y_in,1)
				error('Number of data pairs does not match!');
			end
			AllDataQuantity = size(X_in,2);
			if AllDataQuantity > obj.MaxDataQuantity
				warning(['Too many data pairs! ', ...
					'The data pairs from ',num2str(obj.MaxDataQuantity + 1), ...
					' to ',num2str(AllDataQuantity),' are ignored!']);
				AllDataQuantity = obj.MaxDataQuantity;
			end
			obj.DataQuantity = AllDataQuantity;
			obj.X(:,1:obj.DataQuantity) = X_in(:,1:obj.DataQuantity);
			obj.Y(1:obj.DataQuantity,:) = Y_in(1:obj.DataQuantity,:);
			
			obj.updateParam_Alldata;
		end
		%% Update the parameter for all the data
		function updateParam_Alldata(obj)
			LocalGP_DataQuantity = obj.DataQuantity;
			X_set = obj.X(:,1:LocalGP_DataQuantity);
			for i = 1:obj.y_dim
				y_set = obj.Y(1:LocalGP_DataQuantity,i);

				new_K = obj.kernel(X_set, X_set) + obj.SigmaN .^ 2 * eye(LocalGP_DataQuantity);
				new_L = chol(new_K,'lower');
				new_aux_alpha = new_L \ y_set;
				new_alpha = new_L' \ new_aux_alpha;

				obj.K(1:LocalGP_DataQuantity,1:LocalGP_DataQuantity) = new_K;
				obj.L(1:LocalGP_DataQuantity,1:LocalGP_DataQuantity) = new_L;
				obj.aux_alpha(1:LocalGP_DataQuantity,i) = new_aux_alpha;
				obj.alpha(1:LocalGP_DataQuantity,i) = new_alpha;
			end
% 			obj.xMax = max(X_set,[],2);
% 			obj.xMin = min(X_set,[],2);
		end
		%% Prediction of Mean Value and Variance
		function [mu,var,eta,beta,gamma,eta_min,eta_max] = predict(obj,x)
			LocalGP_DataQuantity = obj.DataQuantity;
			if LocalGP_DataQuantity == 0
				mu = zeros(obj.y_dim,1);
				var = obj.SigmaF ^ 2 * ones(obj.y_dim,1);
			else
				X_set = obj.X(:,1:LocalGP_DataQuantity);
				% Variance
				temp_L = obj.L(1:LocalGP_DataQuantity,1:LocalGP_DataQuantity);
				v = temp_L \ obj.kernel(X_set, x);
				var = obj.kernel(x) - v'*v;
				var = var * ones(obj.y_dim,1);
				% Mean Value
				mu = obj.aux_alpha(1:LocalGP_DataQuantity,:)' * v;
			end
			% Error Bound
			[beta,gamma] = obj.set_ErrorBound(x);
			try
				eta = sqrt(beta) * sqrt(max(var)) + gamma;
				
			catch
				eta = nan;
			end
			eta_min = sqrt(beta) * obj.SigmaN + gamma;
			eta_max = sqrt(beta) * obj.SigmaF + gamma;
		end
		%% Error Bound
		function [beta,gamma] = set_ErrorBound(obj,x)
			LocalGP_DataQuantity = obj.DataQuantity;
			obj.M = prod(ceil((obj.xMax - obj.xMin) * sqrt(obj.x_dim) / (2 * obj.tau)));
			beta = 2 * log(obj.M / (obj.delta / obj.y_dim));
% 			gamma_set = nan(obj.y_dim,1);

			if LocalGP_DataQuantity <= 0
				warning('GP no data!');
				gamma = (0 + obj.Lf) * obj.tau + sqrt(obj.y_dim * beta) * obj.Lsigma * obj.tau;
% 				gamma = nan;
				return;
			end

% 			All_K = obj.K(1:LocalGP_DataQuantity,1:LocalGP_DataQuantity);
% 			obj.omega_SigmaN_tau = 2 * obj.tau * obj.Lk * ...
% 				(1 + LocalGP_DataQuantity * norm(inv(All_K)) * obj.kernel(x));
			for i = 1:obj.y_dim
				alpha_i = obj.alpha(1:LocalGP_DataQuantity,i);
				obj.Lmu_set(i) = obj.Lk * sqrt(LocalGP_DataQuantity) * norm(alpha_i);
% 				gamma_set(i) = (obj.Lmu_set(i) + obj.Lf_set(i)) * obj.tau + sqrt(beta) * obj.Lsigma * obj.tau;
			end
			obj.Lmu = norm(obj.Lmu_set);
			gamma = (obj.Lmu + obj.Lf) * obj.tau + sqrt(obj.y_dim * beta) * obj.Lsigma * obj.tau;
			
		end
		%% Check Saturation
		function is_Saturated = check_Saturation(obj)
			if obj.DataQuantity >= obj.MaxDataQuantity
				is_Saturated = true;
			else
				is_Saturated = false;
			end
		end
		%% 
	end



end