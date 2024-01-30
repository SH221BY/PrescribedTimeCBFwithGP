close all;
% Specify the file names
file1 = 'GP_result.txt';  % Replace with your actual file path

% Load the data
Data = load(file1);
Data = transpose(Data);

IndexOfT = 1;
IndexOfGP_mean = 2:3;
IndexOfErrBd = 4;
IndexOfUn = 5:6;

time = Data(IndexOfT,:);
GP_mean = Data(IndexOfGP_mean,:);
GP_errorBound = Data(IndexOfErrBd,:);
Uncertainty = Data(IndexOfUn,:);
Sub_mu_un = Uncertainty - GP_mean;


Tp1 = 1;
Tp2 = 3;

%% plot
LineWidthjointTra = 2;
LineLimit = 1.5;
learn_color = [0 0 1];
Limit_color = [1.00,0.46,0.00];


figure()
for i = 1:2
    subplot(2,1,i); hold on;
    plot(time,abs(Sub_mu_un(i,:)),'Color',learn_color, 'LineWidth',LineWidthjointTra);
    plot(time, GP_errorBound,'Color',Limit_color, 'LineWidth',LineWidthjointTra);
    xlabel( 'time(sec)' ); ylabel('error'); grid on; 
    %title(sprintf('GP%d result', i)); 
    leg = legend('Prediction error','Error bound');
    set(gca,'linewidth', 1.1,'FontSize',16,'FontName','Times New Roman');
    ax = gca;
    ax.XLabel.FontWeight = 'bold';
    ax.YLabel.FontWeight = 'bold';

end