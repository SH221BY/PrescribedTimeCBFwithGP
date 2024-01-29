close all;
% Specify the file names
file1 = 'h1_desire.txt';  % Replace with your actual file path
file2 = 'h1_learn.txt'; % Replace with your actual file path
file3 = 'h1_uncertainty.txt';  % Replace with your actual file path

% Load the data
Data_desire = load(file1);
Data_learn = load(file2);
Data_uncertainty = load(file3);

IndexOfh1 = 2:5;
IndexOfT = 1;
h1_desire = transpose(Data_desire(:,IndexOfh1));
h1_learn = transpose(Data_learn(:,IndexOfh1));
h1_uncertainty = transpose(Data_uncertainty(:,IndexOfh1));
time = transpose(Data_desire(:,IndexOfT));
T_learn = transpose(Data_learn(:,IndexOfT));
T_uncertainty = transpose(Data_uncertainty(:,IndexOfT));

Tp1 = 1;
Tp2 = 3;

%% plot
learn_color = [0 0 1];
Uncertain_color = [1 0 0 1];
LineWidthjointTra = 2;
LineWidthConstraint = 1.5;
LineX = [0, 3]; LineY = [0,0];
ColorLine = [0.30,0.75,0.93];
LegPos = [0.3156, 0.7923, 0.1375, 0.1179;
          0.7664, 0.6045, 0.1224, 0.1051;
          0.3156, 0.3185, 0.1375, 0.1179;
          0.7675, 0.1311, 0.1238, 0.1029];
figure(); 
for i = 1 : 4
    subplot(2,2,i); hold on;
    % constraint
    line(LineX, LineY, 'Color', ColorLine, 'LineStyle', '--','LineWidth',LineWidthConstraint);
    %data
    plot( time, h1_desire(i,:),'k','LineWidth',LineWidthjointTra ); 
    plot( T_uncertainty, h1_uncertainty(i,:),'Color',Uncertain_color, 'LineWidth',LineWidthjointTra );
    plot( T_learn, h1_learn(i,:),'Color',learn_color, 'LineWidth',LineWidthjointTra );
    xlabel( 'time(sec)' ); ylabel('h1'); grid on; 
    title(sprintf('h1(Constraint %d)', i)); leg = legend('limit','Nominal','PTSC','PTSGPC'); set(leg, 'Position', LegPos(i,:));
end

subplot(2,2,2); leg = legend; 
set(leg, 'Position', legPos2);
%subplot(2,2,4); leg = legend; 
%set(leg, 'Position', legPos);
