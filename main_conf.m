clc;close all;clear;

%% Initial conditions
TrajFlag = 3; %1:safe -> safe, 2: unsafe -> safe 3: unsafe -> safe -> safe
dt = 0.001;
PrescibedTime = 1;
NextPrescribedTinme = 2;
WithoutPTCBFTime = 0;
totalTime = PrescibedTime + NextPrescribedTinme + WithoutPTCBFTime;

UncertaintyFlag = 3; %1: no uncentainty, no GP, 2: uncentainty, no GP, 3: uncentainty and GP, 4: uncetainty and exact uncertainty function

time = 0:dt:totalTime;
TimeLen = length(time);

%% System Parameters
SystemParam = SystemParamInitialization(TrajFlag);

%% total result initialize
result = ResultInitialization(SystemParam.dof, TimeLen);

%% GP off line learning
dof = 2;
MaxDataNum = 900;
LocalGP = OfflineTrainGP(dof, MaxDataNum);

%% Main control loop
LocalPreCBFGP = PreCBFGP_2LinkManipulator(PrescibedTime, NextPrescribedTinme, SystemParam, LocalGP);
t0 = 0;

% learn case
global gSP error_l dt_s PreCBFGP t0_s UncertaintyFlag_s Trajflag_s
gSP = SystemParam; error_l = 0; dt_s = dt; PreCBFGP = LocalPreCBFGP; t0_s=t0; UncertaintyFlag_s = UncertaintyFlag;
Trajflag_s = TrajFlag;
x_cur = [GetCurDesire(t0,0);0;0];
[T_learn, Q_learn] = ode45( @systemDynamics, [time(1), time(end)], x_cur);

% without learn case
x_cur = [GetCurDesire(t0,0);0;0]; UncertaintyFlag = 2;
error_l = 0; UncertaintyFlag_s = UncertaintyFlag;
[T_uncertainty, Q_uncertainty] = ode45( @systemDynamics, [time(1), time(end)], x_cur);

% modify data
Q_learn = transpose(Q_learn);
T_learn = transpose(T_learn);
Q_uncertainty = transpose(Q_uncertainty);
T_uncertainty = transpose(T_uncertainty);
Q_desire = GetCurDesire(time,Q_learn);

%% figure

figure()
hold on
% CBF Constraint plot
LineWidthConstraint = 1.8;


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

LineWidthjointTra = 2;
indices = 1:4:length(Q_desire); % For example, indices of every 10th point
plot3(Q_desire(1,indices), Q_desire(2,indices), time(indices),'.', 'color',[0 0.4470 0.7410]);
plot3(Q_learn(1,:), Q_learn(2,:), T_learn, 'k', Q_uncertainty(1,:), Q_uncertainty(2,:), T_uncertainty, 'LineWidth',LineWidthjointTra);
view(0,90)



xlabel( '$q_1$','Interpreter','latex' ); ylabel( '$q_2$','Interpreter','latex' ); grid on; title('Joint space trajectory'); legend('Nominal','PTSGPC','PTSC','Constratint1','Constratint2','Constratint3','Constratint4', 'Safe region');
set(gca,'linewidth', 1.1,'FontSize',16,'FontName','Times New Roman'); 
ax = gca; % Get the current axes
ax.XColor = 'black'; % Set the x-axis color to black
ax.YColor = 'black'; % Set the y-axis color to black
box on;


% %joint torque and safe torque
% figure()
% subplot(2,1,1)
% plot(time, result.tor_r(1,:), time, result.torsafe_r(1,:), 'LineWidth',2);
% xlabel( 'time(sec)' ); ylabel( 'joint torque(N.m)' ); legend( 'torOrg', 'torSafe' );grid on; title('1st joint torque');
% xlim([0,1.5])
% 
% subplot(2,1,2)
% plot(time, result.tor_r(2,:),time, result.torsafe_r(2,:),'LineWidth',2);
% xlabel( 'time(sec)' ); ylabel( 'joint torque(N.m)' ); legend( 'torNom', 'torSafe' );grid on; title('2nd joint torque');
% xlim([0,1.5])
% 
% %normal torque and safe torque
% figure()
% subplot(2,1,1)
% plot(time, result.tor_org(1,:),'LineWidth',2);
% xlabel( 'time(sec)' ); ylabel( 'joint torque(N.m)' ); grid on; title('1st joint org torque');
% 
% subplot(2,1,2)
% plot(time, result.tor_org(2,:),'LineWidth',2);
% xlabel( 'time(sec)' ); ylabel( 'joint torque(N.m)' ); grid on; title('2nd joint org torque');
% 
% %cartesain pos
% figure(4)
% plot3(result.end_effector_pos_cmd(1,:), result.end_effector_pos_cmd(2,:), time, result.end_effector_pos_r(1,:), result.end_effector_pos_r(2,:),time,'LineWidth',2);
% xlabel( 'xpos(m)' ); ylabel( 'ypos(m)' ); legend( 'cmd', 'feedback' ); grid on; title('planar eff position');
% view(0,90);
% 
% figure(5)
% subplot(2,1,1)
% plot(time, result.end_effector_pos_cmd(1,:), time, result.end_effector_pos_r(1,:),'LineWidth',2);
% xlabel( 'time(sec)' ); ylabel( 'pos(m)' ); legend( 'cmd', 'feedback' ); grid on; title('1st x position');
% 
% subplot(2,1,2)
% plot(time, result.end_effector_pos_cmd(2,:), time, result.end_effector_pos_r(2,:),'LineWidth',2);
% xlabel( 'time(sec)' ); ylabel( 'pos(m)' ); legend( 'cmd', 'feedback' ); grid on; title('2nd y position');
% 
% %joint vel
% figure(6)
% subplot(2,1,1)
% plot(time, result.qdot_r(1,:),'LineWidth',2);
% xlabel( 'time(sec)' ); ylabel( 'joint vel(rad/s)' ); grid on; title('1st joint velocity');
% 
% subplot(2,1,2)
% plot(time, result.qdot_r(2,:),'LineWidth',2);
% xlabel( 'time(sec)' ); ylabel( 'joint pos(rad/s)' ); grid on; title('2nd joint velocity');



