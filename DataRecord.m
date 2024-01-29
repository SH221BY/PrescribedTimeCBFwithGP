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
h1_desire = LocalPreCBFGP.CBFProperty.H_offset*Q_const;
fileID = fopen('h1_desire.txt', 'w');

% Write the data
for i = 1:Len
    fprintf(fileID, '%f\t%f\t%f\t%f\t%f\t%\n', time(i), h1_desire(1, i), h1_desire(2, i), h1_desire(3, i), h1_desire(4, i));
end

% Close the file
fclose(fileID);

%% h1_learn
Len = length(Q_desire);
Q_const = [Q_desire;ones(1,Len)];
h1_desire = LocalPreCBFGP.CBFProperty.H_offset*Q_const;
fileID = fopen('h1_desire.txt', 'w');

% Write the data
for i = 1:Len
    fprintf(fileID, '%f\t%f\t%f\t%f\t%f\t%\n', time(i), h1_desire(1, i), h1_desire(2, i), h1_desire(3, i), h1_desire(4, i));
end

% Close the file
fclose(fileID);