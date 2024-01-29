close all;
% Specify the file names
file1 = 'q_desire.txt';  % Replace with your actual file path
file2 = 'q_learn.txt'; % Replace with your actual file path
file3 = 'q_uncertainty.txt';  % Replace with your actual file path

% Load the data
Data_desire = load(file1);
Data_learn = load(file2);
Data_uncertainty = load(file3);


Q_desire = transpose(Data_desire(:,2:3));
time = transpose(Data_desire(:,1));
Q_learn = transpose(Data_learn(:,2:3));
T_learn = transpose(Data_learn(:,1));
Q_uncertainty = transpose(Data_uncertainty(:,2:3));
T_uncertainty = transpose(Data_uncertainty(:,1));

Tp1 = 1;
Tp2 = 3;
Tol = 0.00001;
%% plot
desire_color = [0 0.4470 0.7410];
Uncertain_color = [0.8500 0.3250 0.0980];


figure(); hold on;
LineWidthjointTra = 2;
indices = 1:4:length(Q_desire); % For example, indices of every 10th point
plot3(Q_desire(1,indices), Q_desire(2,indices), time(indices),'--','Color', desire_color,'LineWidth',LineWidthjointTra);
plot3(Q_uncertainty(1,:), Q_uncertainty(2,:), T_uncertainty,'Color', Uncertain_color, 'LineWidth',LineWidthjointTra);
PaperLine = plot3(Q_learn(1,:), Q_learn(2,:), T_learn, 'k', 'LineWidth',LineWidthjointTra);

view(0,90);


% CBF Constraint plot
LineWidthConstraint = 1.5;


% Plotting x = 0
FirstLineX = [0, 0]; FirstLineY = [-2,2];
line(FirstLineX, FirstLineY, 'Color', 'g', 'LineStyle', '--','LineWidth',LineWidthConstraint);

% Plotting x = pi/2
SecondLineX = [pi/2, pi/2]; SecondLineY = [-2,2];
line(SecondLineX, SecondLineY, 'Color', 'r', 'LineStyle', '--','LineWidth',LineWidthConstraint);


% Plotting x + y = 0 (y = -x)
InclineXleft = -0.4; InclineXright = 2;
x_range = linspace(InclineXleft, InclineXright, 400);
y = -x_range;
plot(x_range, y, 'm--','LineWidth',LineWidthConstraint);

% Plotting x + y = pi/2 (y = pi/2 - x)
y = pi/2 - x_range;
plot(x_range, y, '--', 'color',[0.3010 0.7450 0.9330],'LineWidth',LineWidthConstraint);

% Polygon
P1 = [0,0];
P2 = [pi/2, -pi/2];
P3 = [pi/2, 0];
P4 = [0, pi/2];
PolyX = [P1(1), P2(1), P3(1), P4(1)];
PolyY = [P1(2), P2(2), P3(2), P4(2)];
ColorRGB = [0.35 1 0];
fill(PolyX, PolyY, 	ColorRGB, 'FaceAlpha', 0.3, 'EdgeColor','none'); % 'cyan' is the fill color, 'FaceAlpha' controls transparency


xlabel( '$q_1$','Interpreter','latex' ); ylabel( '$q_2$','Interpreter','latex' ); grid on; 
title('Joint space trajectory'); legend('Nominal','PTSC','PTSGPC','Constratint1','Constratint2','Constratint3','Constratint4', 'Safe region');
set(gca,'linewidth', 1.1,'FontSize',16,'FontName','Times New Roman'); 
ax = gca; % Get the current axes
ax.XColor = 'black'; % Set the x-axis color to black
ax.YColor = 'black'; % Set the y-axis color to black
box on;
uistack(PaperLine, 'top');
%axis square;


% Mark the 500th data point and Add a description (annotation) near the marked point
index_un_pt1 = find(abs(T_uncertainty - Tp1)<=Tol);
index_un_pt2 = find(abs(T_uncertainty - Tp2)<=Tol);
targetPoint_pt1 = Q_uncertainty(:,index_un_pt1);
targetPoint_pt2 = Q_uncertainty(:,index_un_pt2);
plot(targetPoint_pt1(1,1), targetPoint_pt1(2,1), 'o','Color', Uncertain_color); % 'ro' creates a red circle at the point
plot(targetPoint_pt2(1,1), targetPoint_pt1(2,1), 'o','Color', Uncertain_color); % 'ro' creates a red circle at the point
%text(targetPoint(1), targetPoint(2), 't=1 sec', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
normPoint = getNormalizedCoordinates(targetPoint_pt1(:,1), gca);
desc = 'Tp=1sec';
annotation(gcf,'textarrow', [0.7, normPoint(1)], [0.9, normPoint(2)], 'String', desc,'Color', Uncertain_color);