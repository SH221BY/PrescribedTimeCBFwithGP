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

global gSP error_l dt_s PreCBFGP t0_s UncertaintyFlag_s Trajflag_s
gSP = SystemParam; error_l = 0; dt_s = dt; PreCBFGP = LocalPreCBFGP; t0_s=t0; UncertaintyFlag_s = UncertaintyFlag;
Trajflag_s = TrajFlag;
x_cur = [GetCurDesire(t0,0);0;0];
[T, Q] = ode45( @systemDynamics, [time(1), time(end)], x_cur);

% modify data
Q = transpose(Q);
T = transpose(T);
Q_desire = GetCurDesire(time,Q);

%% figure
figure()
set(gca,'FontSize',14,'FontName','Times New Roman');
plot3(Q_desire(1,:), Q_desire(2,:), time, Q(1,:), Q(2,:), T, 'LineWidth',2);
view(0,90)
hold on

% CBF Constraint plot
LineWidthConstraint = 2;


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
plot(x_range, y, 'b--','LineWidth',LineWidthConstraint);

% Polygon
P1 = [0,0];
P2 = [pi/2, -pi/2];
P3 = [pi/2, 0];
P4 = [0, pi/2];
PolyX = [P1(1), P2(1), P3(1), P4(1)];
PolyY = [P1(2), P2(2), P3(2), P4(2)];
ColorRGB = [0.35 1 0];
fill(PolyX, PolyY, 	ColorRGB, 'FaceAlpha', 0.3); % 'cyan' is the fill color, 'FaceAlpha' controls transparency
xlabel( 'Joint 1' ); ylabel( 'Joint 2' ); grid on; title('Joint space trajectory'); legend('Nominal','PTSDGP','Constratint1','Constratint2','Constratint3','Constratint4', 'Safe region');

%% plot result
%joint pos
figure()
subplot(2,1,1)
plot(time, q_desired(1,:), time, result.q_r(1,:),'LineWidth',2);
xlabel( 'time(sec)' ); ylabel( 'joint pos(rad)' ); legend( 'cmd', 'feedback' ); grid on; title('1st joint position');

subplot(2,1,2)
plot(time, q_desired(2,:), time, result.q_r(2,:),'LineWidth',2);
xlabel( 'time(sec)' ); ylabel( 'joint pos(rad)' ); legend( 'cmd', 'feedback' ); grid on; title('2nd joint position');


%% debug code
figure()
subplot(2,1,1)
plot(time,result.a_coe_org_paper(1,:),time,result.a_coe_withUncertainty(1,:),time,result.a_coe_withGP_paper(1,:),'LineWidth',2)
xlabel( 'time(sec)' ); ylabel( 'acoe' ); legend( 'org', 'uncetainty','AfterGP' ); grid on; title('a coe');
xlim([0,1.5])

subplot(2,1,2)
b_coe1 = squeeze(result.b_coe_paper(1,:,:));
plot(time,b_coe1(1,:),time,b_coe1(2,:),'LineWidth',2)
xlabel( 'time(sec)' ); ylabel( 'bcoe' ); legend( 'b1', 'b2' ); grid on; title('b coe');
xlim([0,1.5])

figure()
plot(time,result.CBFCon_with_uncertainty(1,:),time,result.CBFCon_with_GP(1,:),'LineWidth',2)
xlabel( 'time(sec)' ); ylabel( 'CBF_constraint' ); legend( 'uncertainty', 'GP error bound' ); grid on; title('a coe');
xlim([0,1.5])

figure()
plot(time,result.CBFCon_with_uncertainty(1,:) - result.CBFCon_with_GP(1,:), time, result.errorbound,'LineWidth',2)
xlabel( 'time(sec)' ); ylabel( 'error of CBF_constraint' ); grid on; title('check CBF constraint');
xlim([0,1.5]); legend('CBFUncen-CBFGP','errorbound');

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



