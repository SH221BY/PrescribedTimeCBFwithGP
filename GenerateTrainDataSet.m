% q_lim = dof * 2([-max, max])
function [x_train,y_train] = GenerateTrainDataSet(q_lim,dof,SampleNum)

xTrainingDataDim = dof;

datalim = q_lim;
x_train = zeros( xTrainingDataDim, SampleNum );
y_train = zeros( dof, SampleNum );
delta_data = ones( xTrainingDataDim, 1 );


SampleNumInEachDataDim = [sqrt(SampleNum);sqrt(SampleNum)];
SampleDataEachDim = zeros(xTrainingDataDim,max(SampleNumInEachDataDim));

for i = 1:xTrainingDataDim
    delta_data(i) = (datalim(i,2) - datalim(i,1))/(SampleNumInEachDataDim(i)-1);
end

% generate sample data in each dim
for i = 1:xTrainingDataDim
    for j = 1:SampleNumInEachDataDim(i)
        SampleDataEachDim(i,j) = datalim(i,1) + (j-1)*delta_data(i);
    end
end

% generate training data set
data_temp = zeros(xTrainingDataDim,1);

count = 1;
q_dot_sample = ones( 2, SampleNum);
for i = 1:SampleNumInEachDataDim(1)
    data_temp(1,1) = -9 + (i-1)*10/(SampleNumInEachDataDim(1)-1);
    for j = 1:SampleNumInEachDataDim(2)
        data_temp(2,1) = -15 + (j-1)*22/(SampleNumInEachDataDim(1)-1);
        q_dot_sample(:,count) = data_temp;
        count = count + 1;
    end
end

x_train = [x_train;q_dot_sample ];

count = 1;
for i = 1:SampleNumInEachDataDim(1)
    data_temp(1,1) = datalim(1,1) + (i-1)*delta_data(1);
    for j = 1:SampleNumInEachDataDim(2)
        data_temp(2,1) = datalim(2,1) + (j-1)*delta_data(2);
        x_train(1:2,count) = data_temp;
        y_train(:,count) = GenerateUncertainty(data_temp,x_train(3:4,count));
        count = count + 1;
    end
end

end