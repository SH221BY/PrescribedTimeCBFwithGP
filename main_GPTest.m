X_min=-1;
X_max = 20;


x_train=[1,2,3,4,5,6,7,8,9,10]; % dim * dataNum
x_now = 9.99;
%x_train = [x_train;x_train];
%x_now = [9.99;9.99];
y_train=[2,4,6,8,10,12,14,16,18,20];
y_train = transpose(y_train);  % dataNum * dim

m = size(x_train,1); %dim
n = size(x_train,2); %dataNum
xdim = size(x_train,1);
ydim = size(y_train,2);
MaxDataNum = 200;
SigmaF = 1;
SigmaL = 0.2 * ones(n,1);
SigmaN = 0.01;

LocalGP = LocalGP_MultiOutput(xdim,ydim,MaxDataNum,SigmaN,SigmaF,SigmaL);

LocalGP.add_Alldata(x_train, y_train);

LocalGP.xMin = X_min;
LocalGP.xMax = X_max;


[mu,var,eta,beta,gamma,eta_min] = LocalGP.predict(x_now);