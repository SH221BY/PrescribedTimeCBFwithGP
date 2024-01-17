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
%% PreCBF initialization
PreCBFParam = PreCBFParamInitialization();

%% GP off line learning
dof = size(q,1);
MaxDataNum = 900;
LocalGP = OfflineTrainGP(dof, MaxDataNum);

%% Main control loop
q = [q_desired(1,1); q_desired(2,1)];
q_dot = [0; 0];
PrescribedTimeFlag = 1; % 1: preCBF, 0: without PreCBF
UncertaintyFlag = 3; %1: no uncentainty, no GP, 2: uncentainty, no GP, 3: uncentainty and GP, 4: uncetainty and uncertainty function

for i = 1:TimeLen
    [u_norm, error] = PDController(q_desired(:,i), q, q_dot, SystemParam);
    % Prescribed time CBF
    [u_safe, Nom_M ,Nom_C, Nom_G,result.a_coe_org_paper(1,i),result.a_coe_withUncertainty(1,i),result.a_coe_withGP_paper(1,i),result.b_coe_paper(:,i), result.errorbound(1,i)] = ...
        PresrcibedTime_CBF(u_norm,q,q_dot,time(i),PreCBFParam,SystemParam,LocalGP,UncertaintyFlag);
    
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
    result.CBFCon_with_uncertainty(1,i) = result.a_coe_withUncertainty(1,i) + dot(result.b_coe_paper(:,i), u_safe);
    result.CBFCon_with_GP(1,i) = result.a_coe_withGP_paper(1,i) + dot(result.b_coe_paper(:,i), u_safe);
end

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
plot(time,result.a_coe_org_paper(1,:),time,result.a_coe_withUncertainty,time,result.a_coe_withGP_paper(1,:),'LineWidth',2)
xlabel( 'time(sec)' ); ylabel( 'acoe' ); legend( 'org', 'uncetainty','AfterGP' ); grid on; title('a coe');
xlim([0,1.5])

subplot(2,1,2)
plot(time,result.b_coe_paper(1,:),time,result.b_coe_paper(2,:),'LineWidth',2)
xlabel( 'time(sec)' ); ylabel( 'bcoe' ); legend( 'b1', 'b2' ); grid on; title('b coe');
xlim([0,1.5])

figure()
plot(time,result.CBFCon_with_uncertainty,time,result.CBFCon_with_GP,'LineWidth',2)
xlabel( 'time(sec)' ); ylabel( 'CBF_constraint' ); legend( 'uncertainty', 'GP error bound' ); grid on; title('a coe');
xlim([0,1.5])

figure()
plot(time,result.CBFCon_with_uncertainty - result.CBFCon_with_GP, time, result.errorbound,'LineWidth',2)
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



