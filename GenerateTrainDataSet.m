function [x_train,y_train] = GenerateTrainDataSet(q_lim, q_dot_lim, MaxSampleNum,dof)
x_train = zeros( dof, MaxSampleNum );
y_train = zeros( dof, MaxSampleNum );
Data_dis1 = q_lim(2) - q_lim(1);
Data_dis2 = q_dot_lim(2) - q_dot_lim(1);

a=(1-MaxSampleNum);
b = Data_dis1+Data_dis2;
c = Data_dis1*Data_dis2;

delta_data = max((-b+sqrt(b^2-4*a*c))/(2*a), (-b-sqrt(b^2-4*a*c))/(2*a));



end