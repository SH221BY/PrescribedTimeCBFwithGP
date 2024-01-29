close all;
% Specify the file names
file1 = 'q_desire.txt';  % Replace with your actual file path
file2 = 'q_learn.txt'; % Replace with your actual file path
file3 = 'q_uncertainty.txt';  % Replace with your actual file path

% Load the data
Data_desire = load(file1);
Data_learn = load(file2);
Data_uncertainty = load(file3);

IndexOfQ = 2:3;
IndexOfT = 1;
Q_desire = transpose(Data_desire(:,IndexOfQ));
time = transpose(Data_desire(:,IndexOfT));
Q_learn = transpose(Data_learn(:,IndexOfQ));
T_learn = transpose(Data_learn(:,IndexOfT));
Q_uncertainty = transpose(Data_uncertainty(:,IndexOfQ));
T_uncertainty = transpose(Data_uncertainty(:,IndexOfT));

Tp1 = 1;
Tp2 = 3;
Tol = 0.0001;
%% plot
figure(); hold on;

%% CBF Constraint plot
LineWidthConstraint = 1.5;


% Plotting x = 0
FirstLineX = [0, 0]; FirstLineY = [-2,2];
ColorFirstLixeX = [0.75,0.29,0.09];
line(FirstLineX, FirstLineY, 'Color', ColorFirstLixeX, 'LineStyle', '--','LineWidth',LineWidthConstraint);

% Plotting x = pi/2
SecondLineX = [pi/2, pi/2]; SecondLineY = [-2,2];
ColorSecondLixeX = [0.30,0.75,0.93];
line(SecondLineX, SecondLineY, 'Color', ColorSecondLixeX, 'LineStyle', '--','LineWidth',LineWidthConstraint);


% Plotting x + y = 0 (y = -x)
InclineXleft = -0.4; InclineXright = 2;
ColorInclineXleft = [1.00,0.46,0.00];
x_range = linspace(InclineXleft, InclineXright, 400);
y = -x_range;
plot(x_range, y, 'Color', ColorInclineXleft, 'LineStyle', '--','LineWidth',LineWidthConstraint);

% Plotting x + y = pi/2 (y = pi/2 - x)
ColorInclineXright = [0.00,0.80,0.00];
y = pi/2 - x_range;
plot(x_range, y, 'Color',ColorInclineXright,'LineStyle', '--','LineWidth',LineWidthConstraint);

% Polygon
P1 = [0,0];
P2 = [pi/2, -pi/2];
P3 = [pi/2, 0];
P4 = [0, pi/2];
PolyX = [P1(1), P2(1), P3(1), P4(1)];
PolyY = [P1(2), P2(2), P3(2), P4(2)];
PolyColor = [0.35 1 0];
fill(PolyX, PolyY, 	PolyColor, 'FaceAlpha', 0.4, 'EdgeColor','none'); % 'cyan' is the fill color, 'FaceAlpha' controls transparency


%% data trajectory
learn_color = [0 0 1];
Uncertain_color = [1 0 0 1];


LineWidthjointTra = 2;
LineWidthLearn = 3;
plot(Q_desire(1,:), Q_desire(2,:),'k','LineWidth',LineWidthjointTra);
plot(Q_uncertainty(1,:), Q_uncertainty(2,:),'Color', Uncertain_color, 'LineWidth',LineWidthjointTra);
plot(Q_learn(1,:), Q_learn(2,:),'Color', learn_color, 'LineWidth',LineWidthLearn);

%axis square;


% Mark the critical point
index_un_pt1 = find(abs(T_uncertainty - Tp1)<=Tol);
index_un_pt2 = find(abs(T_uncertainty - Tp2)<=Tol);
index_gp_pt1 = find(abs(T_learn - Tp1)<=Tol);
index_gp_pt2 = find(abs(T_learn - Tp2)<=Tol);
taPt_un_pt1 = Q_uncertainty(:,index_un_pt1);
taPt_un_pt2 = Q_uncertainty(:,index_un_pt2);
taPt_gp_pt1 = Q_learn(:,index_gp_pt1);
taPt_gp_pt2 = Q_learn(:,index_gp_pt2);
plot(taPt_un_pt1(1,1), taPt_un_pt1(2,1), '+','Color', Uncertain_color,'LineWidth',LineWidthjointTra); % 'ro' creates a red circle at the point
plot(taPt_un_pt2(1,1), taPt_un_pt2(2,1), '+','Color', Uncertain_color,'LineWidth',LineWidthjointTra); % 'ro' creates a red circle at the point
plot(taPt_gp_pt1(1,1), taPt_gp_pt1(2,1), '+','Color', learn_color,'LineWidth',LineWidthLearn); % 'ro' creates a red circle at the point
plot(taPt_gp_pt2(1,1), taPt_gp_pt2(2,1), '+','Color', learn_color,'LineWidth',LineWidthLearn); % 'ro' creates a red circle at the point

xlabel( '$\mathbf{q_1}$(\textbf{rad})','Interpreter','latex' ); ylabel( '$\mathbf{q_2}$(\textbf{rad})','Interpreter','latex' ); grid on; 
title('Joint space trajectory'); leg = legend('Constratint1','Constratint2','Constratint3','Constratint4', 'Safe region','Nominal','PTSC','PTSGPC'); 
legPos = [0.6795, 0.1381, 0.2009, 0.3321]; set(leg, 'Position', legPos);
set(gca,'linewidth', 1.1,'FontSize',16,'FontName','Times New Roman'); 
xlimit = [-1.8203, 4.1797]; ylimit=[-2.5434, 3.4566]; xlim(xlimit); ylim(ylimit);
ax = gca; % Get the current axes
ax.XColor = 'black'; % Set the x-axis color to black
ax.YColor = 'black'; % Set the y-axis color to black
box on;
% %text(targetPoint(1), targetPoint(2), 't=1 sec', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
% normPoint = getNormalizedCoordinates(targetPoint_pt1(:,1), gca);
% desc = 'Tp=1sec';
% annotation(gcf,'textarrow', [0.7, normPoint(1)], [0.9, normPoint(2)], 'String', desc,'Color', Uncertain_color);