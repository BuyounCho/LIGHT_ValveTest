clc; clear all;
close all;

D2R = pi/180;
R2D = 180/pi;
mm3s_to_LPM = 6/100000.0;

addpath('ValvePerfTest_Data')
list = dir(fullfile('ValvePerfTest_Data','*.txt'));
filename = {list.name};
filename_sorted = sort(filename);
filename_lastest = filename_sorted{length(filename)-0}
Data_temp = load(filename_lastest);
dt = Data_temp(1,1);
Data_AL = Data_temp(:,2:end);

Joint_Selected = filename_lastest(end-6:end-4);

[m,n] = size(Data_AL);

t = (0:dt:((m-1)*dt))';

% % Joint Data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ActPos_Ref     = Data_AL(:,1);
ActPos         = Data_AL(:,2);
ActVel_Ref     = Data_AL(:,3);
ActVel         = Data_AL(:,4);
ActForce_Ref    = Data_AL(:,5);
ActForce        = Data_AL(:,6);

ValvePos_Ref     = Data_AL(:,7);
ValvePos_BoardRef = Data_AL(:,8);
ValvePos         = Data_AL(:,9);

PWM_Ref         = Data_AL(:,10);
PWM             = Data_AL(:,11);

DebugData1      = Data_AL(:,12);
DebugData2      = Data_AL(:,13);
DebugData3      = Data_AL(:,14);
DebugData4      = Data_AL(:,15);

DebugData1_AL      = Data_AL(:,16);
DebugData2_AL      = Data_AL(:,17);
DebugData3_AL      = Data_AL(:,18);
%DebugData4_AL      = Data_AL(:,19);

% %% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Figure number : 0xxx

figure(101)
plot(t,ActPos_Ref,t,ActPos);
title('Actuator Position Control Performance')
legend('Ref.','Meas.');
%  
%figure(102)
%plot(t,ActVel_Ref,t,ActVel);
%title('Actuator Velocity Control Performance')
%legend('Ref.','Meas.');

figure(103)
plot(t,ActForce_Ref,t,ActForce);
title('Actuator Force Control Performance')
legend('Ref.','Meas.');

figure(104)
plot(t,ValvePos_Ref,t,ValvePos);
title('Valve Opening')
legend('Ref.','Meas.');

figure(105)
plot(t,PWM);
title('Valve Voltage')

%figure(105)
%plot(t,DebugData1_AL);
%figure(106)
%plot(t,DebugData2_AL);
%figure(107)
%plot(t,DebugData3_AL);
%

