clc;close all;

%% System Parameters
SystemParam = SystemParamInitialization();

%% Initial conditions
[q_desired, time, ~] = jointTrajectoryGenerator(SystemParam.totalTime, SystemParam.dt);
q = [q_desired(1,1); q_desired(2,1)];
q_dot = [0; 0];
TimeLen = length(q_desired);
base_pos = [0;0];

%% total result initialize
result = ResultInitialization(SystemParam.dof, TimeLen);
% normal loop
result.tor_org = GetNormalWithoutPreCBF(TimeLen, q_desired, q, q_dot, SystemParam);
%% PreCBF
PreCBFParam = PreCBFParamInitialization();

%% Main control loop
q = [q_desired(1,1); q_desired(2,1)];
q_dot = [0; 0];
PrescribedTimeFlag = 1; % 1: preCBF, 0: without PreCBF

for i = 1:TimeLen
    [u_norm, error] = PDController(q_desired(:,i), q, q_dot, SystemParam);
    % Prescribed time CBF
    [u_safe, Nom_M ,Nom_C, Nom_G] = PresrcibedTime_CBF(u_norm,q,q_dot,time(i),PreCBFParam,SystemParam);
    
    if (PrescribedTimeFlag == 1 && time(i) == PreCBFParam.PrescribedTime)
        PreCBFParam.u_terminal = GetUTerminalWithPreCBF(Nom_M,u_norm);
    end

    % input to real system
    if (PrescribedTimeFlag == 0)
        [q, q_dot] = dynamicSystem(u_norm, q, q_dot,SystemParam);
    else %flag = 1
        [q, q_dot] = dynamicSystem(u_safe, q, q_dot, SystemParam);
    end

    % record result
    result.q_r(:,i) = q;
    result.qdot_r(:,i) = q_dot;
    result.tor_r(:,i) = u_norm;
    result.torsafe_r(:,i) = u_safe;
    result.q_error_r(:,i)=error;
    result.end_effector_pos_cmd(:,i) = forwardKinematics(SystemParam, q_desired(:,i));
    result.end_effector_pos_r(:,i) = forwardKinematics(SystemParam, q_desired(:,i));
end

%% plot result
%joint pos
figure(1)
subplot(2,1,1)
plot(time, q_desired(1,:), time, result.q_r(1,:),'LineWidth',2);
xlabel( 'time(sec)' ); ylabel( 'joint pos(rad)' ); legend( 'cmd', 'feedback' ); grid on; title('1st joint position');

subplot(2,1,2)
plot(time, q_desired(2,:), time, result.q_r(2,:),'LineWidth',2);
xlabel( 'time(sec)' ); ylabel( 'joint pos(rad)' ); legend( 'cmd', 'feedback' ); grid on; title('2nd joint position');

%joint torque and safe torque
figure(2)
subplot(2,1,1)
plot(time, result.tor_r(1,:), time, result.torsafe_r(1,:), 'LineWidth',2);
xlabel( 'time(sec)' ); ylabel( 'joint torque(N.m)' ); legend( 'torOrg', 'torSafe' );grid on; title('1st joint torque');

subplot(2,1,2)
plot(time, result.tor_r(2,:),time, result.torsafe_r(2,:),'LineWidth',2);
xlabel( 'time(sec)' ); ylabel( 'joint torque(N.m)' ); legend( 'torNom', 'torSafe' );grid on; title('2nd joint torque');

%normal torque and safe torque
figure(3)
subplot(2,1,1)
plot(time, result.tor_org(1,:),'LineWidth',2);
xlabel( 'time(sec)' ); ylabel( 'joint torque(N.m)' ); grid on; title('1st joint org torque');

subplot(2,1,2)
plot(time, result.tor_org(2,:),'LineWidth',2);
xlabel( 'time(sec)' ); ylabel( 'joint torque(N.m)' ); grid on; title('2nd joint org torque');

%cartesain pos
figure(4)
plot3(result.end_effector_pos_cmd(1,:), result.end_effector_pos_cmd(2,:), time, result.end_effector_pos_r(1,:), result.end_effector_pos_r(2,:),time,'LineWidth',2);
xlabel( 'xpos(m)' ); ylabel( 'ypos(m)' ); legend( 'cmd', 'feedback' ); grid on; title('planar eff position');
view(0,90);

figure(5)
subplot(2,1,1)
plot(time, result.end_effector_pos_cmd(1,:), time, result.end_effector_pos_r(1,:),'LineWidth',2);
xlabel( 'time(sec)' ); ylabel( 'pos(m)' ); legend( 'cmd', 'feedback' ); grid on; title('1st x position');

subplot(2,1,2)
plot(time, result.end_effector_pos_cmd(2,:), time, result.end_effector_pos_r(2,:),'LineWidth',2);
xlabel( 'time(sec)' ); ylabel( 'pos(m)' ); legend( 'cmd', 'feedback' ); grid on; title('2nd y position');


