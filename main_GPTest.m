dof = 2;
MaxDataNum = 400;
LocalGP = OfflineTrainGP(dof, MaxDataNum);

%% validation
SystemParam = SystemParamInitialization();
[x_test, time, ~] = jointTrajectoryGenerator(SystemParam.totalTime, SystemParam.dt);
y_test = GenerateUncertainty(x_test);

% test
mu = zeros(dof, length(time));
var = zeros(dof, length(time));
beta = zeros(dof, length(time));
gamma = zeros(dof, length(time));
for i = 1:length(time)
    [mu(:,i),var(:,i),eta,beta,gamma,eta_min] = LocalGP.predict(x_test(:,i));
end

upper_bound = mu + var;
lower_bound = mu - var;

upper_errorbound = mu + eta;
low_errorbound = mu - eta;

% plot
figure()
subplot(2,1,1)
plot(time,y_test(1,:),time,mu(1,:),'LineWidth',2) ;
hold on;
fill([time, fliplr(time)], [upper_bound(1,:), fliplr(lower_bound(1,:))], 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
fill([time, fliplr(time)], [upper_errorbound(1,:), fliplr(low_errorbound(1,:))], 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
xlabel( 'time(sec)' ); ylabel( 'uncertainty' ); legend( 'real', 'GP' ); grid on; title('1st GP');

subplot(2,1,2)
plot(time,y_test(2,:),time,mu(2,:),'LineWidth',2) ;
hold on;
fill([time, fliplr(time)], [upper_bound(2,:), fliplr(lower_bound(2,:))], 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
fill([time, fliplr(time)], [upper_errorbound(2,:), fliplr(low_errorbound(2,:))], 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
xlabel( 'time(sec)' ); ylabel( 'uncertainty' ); legend( 'real', 'GP' ); grid on; title('2nd GP');