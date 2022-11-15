clc; clear all;
close all; 

D2R = pi/180;
R2D = 180/pi;
mm3s_to_LPM = 6/100000.0;

addpath('PumpControl_AL_Data')
list = dir(fullfile('PumpControl_AL_Data','*.txt'));
filename = {list.name};
filename_sorted = sort(filename);
filename_lastest = filename_sorted{length(filename)-0}

Data_temp = load(filename_lastest);
dt = Data_temp(1,1);
Data_AL = Data_temp(:,2:end);

[m,n] = size(Data_AL);
% [k,l] = size(Data_DAEMON);

%t = (0:dt:((m-1)*dt))';

% Data Parsing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t          = Data_AL(:,11);
Ps_SIM     = Data_AL(:,12);
Psdes_SIM  = Data_AL(:,13);
wp_SIM     = Data_AL(:,14);
Qdes_SIM   = Data_AL(:,15);
CalcTime   = Data_AL(:,16);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;

figure(1);
plot(t,Psdes_SIM,t,Ps_SIM);
legend('Reference','State');

figure(2);
plot(t,wp_SIM);
legend('Pump Speed');

figure(3);
plot(t,CalcTime);
legend('Calculation Time');


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % close all;
% fig7 = figure(7); hold on;
% grid on;
% plot_Qflow = plot(t,Qflow_JointVel(:,1:6),'LineWidth',1.5); 
% 
% % Axes properties : https://kr.mathworks.com/help/matlab/ref/matlab.graphics.axis.axes-properties.html
% ax_fig7 = fig7.CurrentAxes;
% ax_fig7.FontName = 'Arial';
% ax_fig7.FontSize = 12;
% ax_fig7.Position = [0.150,0.13,0.800,0.80];
% % ax_fig7.XLim = [45 60];
% % ax_fig7.YLim = [0 2300];
% % ax_fig7.YColor = [0 0 0];
% % ax_fig7.Title.String = 'Supply Pressure';
% % ax_fig7.Title.FontSize = 13;
% ax_fig7.XLabel.String = 'Time (sec)';
% ax_fig7.XLabel.FontSize = 13;
% ax_fig7.YLabel.String = 'Flowrate (LPM)';
% ax_fig7.YLabel.FontSize = 13;
% 
% legend( [plot_Qflow] , {'RHR','RHY','RHP','RKN','RA1','RA2'} );
% ax_fig7.Legend.NumColumns = 1;
% ax_fig7.Legend.Location = 'northeast';
% % ax_fig7.Legend.FontSize = 14;
% 
% fig8 = figure(8); hold on;
% grid on;
% plot_Qflow_Ank1 = plot(t,Qflow_JointVel(:,5:6),'LineWidth',1.5); 
% plot_Qflow_Ank2 = plot(t,Qflow_JointVel(:,11:12),'LineWidth',1.5); 
% 
% % Axes properties : https://kr.mathworks.com/help/matlab/ref/matlab.graphics.axis.axes-properties.html
% ax_fig8 = fig8.CurrentAxes;
% ax_fig8.FontName = 'Arial';
% ax_fig8.FontSize = 12;
% ax_fig8.Position = [0.150,0.13,0.800,0.80];
% ax_fig8.XLabel.String = 'Time (sec)';
% ax_fig8.XLabel.FontSize = 13;
% ax_fig8.YLabel.String = 'Flowrate (LPM)';
% ax_fig8.YLabel.FontSize = 13;
% 
% legend( [plot_Qflow_Ank1; plot_Qflow_Ank2] , {'RA1','RA2','LA1','LA2'} );
% ax_fig8.Legend.NumColumns = 1;
% ax_fig8.Legend.Location = 'northeast';
% % ax_fig7.Legend.FontSize = 14;
%

%% 

