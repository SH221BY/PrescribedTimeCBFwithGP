clc;

%% Q_desire
fileID = fopen('q_desire.txt', 'w');

Len = length(Q_desire);
% Write the data
for i = 1:Len
    fprintf(fileID, '%f\t%f\t%f\t\n', time(i), Q_desire(1, i), Q_desire(2, i));
end

% Close the file
fclose(fileID);

%% Q_learn
fileID = fopen('q_learn.txt', 'w');

Len = length(Q_learn);
% Write the data
for i = 1:Len
    fprintf(fileID, '%f\t%f\t%f\t\n', T_learn(i), Q_learn(1, i), Q_learn(2, i));
end

% Close the file
fclose(fileID);

%% Q_uncertainty
fileID = fopen('q_uncertainty.txt', 'w');

Len = length(Q_uncertainty);
% Write the data
for i = 1:Len
    fprintf(fileID, '%f\t%f\t%f\t\n', T_uncertainty(i), Q_uncertainty(1, i), Q_uncertainty(2, i));
end

% Close the file
fclose(fileID);

%% h1_desire
Len = length(Q_desire);
Q_const = [Q_desire;ones(1,Len)];
h1 = LocalPreCBFGP.CBFProperty.H_offset*Q_const;
fileID = fopen('h1_desire.txt', 'w');

% Write the data
for i = 1:Len
    fprintf(fileID, '%f\t%f\t%f\t%f\t%f\t\n', time(i), h1(1, i), h1(2, i), h1(3, i), h1(4, i));
end

% Close the file
fclose(fileID);

%% h1_learn
Len = length(Q_learn);
Q_const = [Q_learn;ones(1,Len)];
h1 = LocalPreCBFGP.CBFProperty.H_offset*Q_const;
fileID = fopen('h1_learn.txt', 'w');

% Write the data
for i = 1:Len
    fprintf(fileID, '%f\t%f\t%f\t%f\t%f\t\n', T_learn(i), h1(1, i), h1(2, i), h1(3, i), h1(4, i));
end

% Close the file
fclose(fileID);

%% h1_un
Len = length(Q_uncertainty);
Q_const = [Q_uncertainty;ones(1,Len)];
h1 = LocalPreCBFGP.CBFProperty.H_offset*Q_const;
fileID = fopen('h1_uncertainty.txt', 'w');

% Write the data
for i = 1:Len
    fprintf(fileID, '%f\t%f\t%f\t%f\t%f\t\n', T_uncertainty(i), h1(1, i), h1(2, i), h1(3, i), h1(4, i));
end

% Close the file
fclose(fileID);

%% GP
Len = length(Q_learn);
un_r = zeros(2, Len);
mu_r = zeros(2, Len);
eta_r = zeros(1, Len);
fileID = fopen('GP_result.txt', 'w');
for i = 1:Len
    un_r(:,i) = GenerateUncertainty(Q_learn(1:2,i));
    [mu_r(:,i),var,eta_r(:,i),beta,gamma,eta_min] = LocalGP.predict(Q_learn(1:2,i));
    fprintf(fileID, '%f\t%f\t%f\t%f\t%f\t%f\t\n', T_learn(i), mu_r(1, i), mu_r(2, i), eta_r(i), un_r(1, i), un_r(2, i));
end

% Close the file
fclose(fileID);
