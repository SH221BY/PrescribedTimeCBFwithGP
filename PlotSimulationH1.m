close all;
% Specify the file names
file1 = 'h1_desire.txt';  % Replace with your actual file path
file2 = 'h1_learn.txt'; % Replace with your actual file path
file3 = 'h1_uncertainty.txt';  % Replace with your actual file path

% Load the data
Data_desire = transpose(load(file1));
Data_learn = transpose(load(file2));
Data_uncertainty = transpose(load(file3));

IndexOfh1 = 2:5;
IndexOfT = 1;
h1_desire = Data_desire(IndexOfh1,:);
h1_learn = Data_learn(IndexOfh1,:);
h1_uncertainty = Data_uncertainty(IndexOfh1,:);
time = Data_desire(IndexOfT,:);
T_learn = Data_learn(IndexOfT,:);
T_uncertainty = Data_uncertainty(IndexOfT,:);

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
          0.7478, 0.5899, 0.1495, 0.1344;
          0.3156, 0.3185, 0.1375, 0.1179;
          0.7479, 0.1154, 0.1495, 0.1344];
figure(); 

P1 = [0, 0];
P2 = [3, 0];
P3 = [3, 4];
P4 = [0, 4];
PolyX = [Tp1, Tp2, Tp2, Tp1;
         Tp1, Tp2, Tp2, Tp1;
         Tp1, Tp2, Tp2, Tp1;
         Tp1, Tp2, Tp2, Tp1;];

PolyY = [0, 0, 4, 4;
         0, 0, 4, 4;
         0, 0, 8, 8;
         0, 0, 5, 5;];
PolyColor = [0.2 1 0];

for i = 1 : 4
    subplot(2,2,i); hold on;
    % constraint
    line(LineX, LineY, 'Color', ColorLine, 'LineStyle', '--','LineWidth',LineWidthConstraint);
    %data
    fill(PolyX(i,:), PolyY(i,:), 	PolyColor, 'FaceAlpha', 0.4, 'EdgeColor','none');
    plot( time, h1_desire(i,:),'k','LineWidth',LineWidthjointTra ); 
    plot( T_uncertainty, h1_uncertainty(i,:),'Color',Uncertain_color, 'LineWidth',LineWidthjointTra );
    plot( T_learn, h1_learn(i,:),'Color',learn_color, 'LineWidth',LineWidthjointTra );
    xlabel( 'time(sec)' ); ylabel('h1'); grid on; 
    title(sprintf('Constraint %d', i)); leg = legend('Limit','Safe region','Nominal','PTSC','PTSGPC'); set(leg, 'Position', LegPos(i,:));
    set(gca,'linewidth', 1.1,'FontSize',16,'FontName','Times New Roman');
    ax = gca;
    ax.XLabel.FontWeight = 'bold';
    ax.YLabel.FontWeight = 'bold';
end

fig = gcf;  % Gets the current figure

% Save using -bestfit option
set(fig, 'PaperPositionMode', 'auto');
fig.Position = [100 100 720 540];
print(fig, 'MyFigure.pdf', '-dpdf');


