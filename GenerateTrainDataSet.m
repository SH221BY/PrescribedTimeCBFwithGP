% q_lim = dof * 2([-max, max])
% q_lim = dof * 2([-max, max])
function [x_train,y_train] = GenerateTrainDataSet(q_lim, q_dot_lim,dof)

SampleNum = 400;
xDataDim = dof * 2;
x_train = zeros( xDataDim, SampleNum );
y_train = zeros( dof, SampleNum );
delta_data = ones( xDataDim, 1 );
datalim = [q_lim;q_dot_lim];


SampleNumInEachDataDim = [4;4;5;5];
SampleDataEachDim = zeros(xDataDim,max(SampleNumInEachDataDim));

for i = 1:xDataDim
    delta_data(i) = (datalim(i,2) - datalim(i,1))/(SampleNumInEachDataDim(i)-1);
end

% generate sample data in each dim
for i = 1:xDataDim
    for j = 1:SampleNumInEachDataDim(i)
        SampleDataEachDim(i,j) = datalim(i,1) + (j-1)*delta_data(i);
    end
end


end