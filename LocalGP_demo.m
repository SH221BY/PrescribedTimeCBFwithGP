do_use_InitialDataSet = false;
% GP setting 
MaxDataQuantity = 200;
SigmaF = 1;
SigmaL = 0.2 * ones(n,1);
SigmaN = 0.01;
X_min = [-1.5,-1.5];
X_max = [ 1.5, 1.5];
LocalGP = LocalGP_MultiOutput(m * n,n,MaxDataQuantity, ...
	SigmaN,SigmaF,SigmaL);

if do_use_InitialDataSet
	Pre_DataSet_Quantity = 100;
	X_in = transpose(X_min + X_max) / 2 + ...
		(rand(m * n,Pre_DataSet_Quantity) - 0.5) .* transpose(abs(X_max - X_min));
	Y_in = GPmeetsCD_UnknownDynamics(X_in);
	LocalGP.add_Alldata(X_in,Y_in);
else
	Pre_DataSet_Quantity = 1;
	X_in = kron(ones(1,Pre_DataSet_Quantity),zeros(m * n,1));
	Y_in = GPmeetsCD_UnknownDynamics(X_in);
	LocalGP.add_Alldata(X_in,Y_in);
end
LocalGP.xMin = X_min;
LocalGP.xMax = X_max;

%GP online learning and prediction with data deletion
if LocalGP.check_Saturation
	LocalGP.downdateParam(1);
end
LocalGP.addPoint(x_now,y_now);
[mu,var,eta,beta,gamma,eta_min] = LocalGP.predict(x_now);