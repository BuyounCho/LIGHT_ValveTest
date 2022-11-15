clc; clear all;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FolderLists = dir('2021*Both');
Path_LatestID = FolderLists(end).name;
% Path_LatestID = '20210331_135609_Good';
if(isempty(FolderLists)) 
    error('There is no data with both direction!');
end
    
addpath(Path_LatestID);

FolderLists_Direction = dir(Path_LatestID);
Path_Positive = FolderLists_Direction(end).name;
Path_Positive = strcat(Path_LatestID,'/',Path_Positive);
addpath(Path_Positive);
Path_Negative = FolderLists_Direction(end-1).name;
Path_Negative = strcat(Path_LatestID,'/',Path_Negative);

addpath(Path_Negative);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Positive direction ID
FolderLists_Pressure = dir(strcat(Path_Positive,'/Ps*'));
n_Pressure = length(FolderLists_Pressure);
Path_Pressure = FolderLists_Pressure(1).name;
Path_Pressure = strcat(Path_Positive,'/',Path_Pressure);
addpath(Path_Pressure);
FolderLists_Opening = dir(strcat(Path_Pressure,'/OPEN*'));
n_Opening = length(FolderLists_Opening);

PressureSet_Positive = zeros(1,n_Pressure);
OpeningSet_Positive = zeros(n_Opening,n_Pressure);
FlowRateSet_Positive = zeros(n_Opening,n_Pressure);

for i = 1:n_Pressure
    PressureSet_Positive(i) = sscanf(FolderLists_Pressure(i).name, strcat('Ps_','%d','bar'));
    Path_Pressure = FolderLists_Pressure(i).name;
    Path_Pressure = strcat(Path_Positive,'/',Path_Pressure);
    addpath(Path_Pressure);
    
    FolderLists_Opening = dir(strcat(Path_Pressure,'/OPEN*'));
    n_Opening = length(FolderLists_Opening);
    
    
    for j = 1:n_Opening
        OpeningName = FolderLists_Opening(j).name;
        Data_temp = load(strcat(Path_Pressure,'/',OpeningName));
        dt = Data_temp(1,1);
        Data_AL = Data_temp(:,2:end);
        
        [m,n] = size(Data_AL);
        t = (0:dt:((m-1)*dt))';
        
        Angle           = Data_AL(:,1);
        AngVel          = Data_AL(:,2);
        ValveOpen       = Data_AL(:,3);
        ValveOpenRef    = Data_AL(:,4);
        PumpPres        = Data_AL(:,5);
        
        rA = 9595.0;
        FlowRateSet_Positive(j,i) = mean(AngVel)/180.0*pi*rA*1e-6*60;
        OpeningSet_Positive(j,i) = mean(ValveOpen);
    end
end

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Negative direction ID
FolderLists_Pressure = dir(strcat(Path_Negative,'/Ps*'));
n_Pressure = length(FolderLists_Pressure);
Path_Pressure = FolderLists_Pressure(1).name;
Path_Pressure = strcat(Path_Negative,'/',Path_Pressure);
addpath(Path_Pressure);
FolderLists_Opening = dir(strcat(Path_Pressure,'/OPEN*'));
n_Opening = length(FolderLists_Opening);

PressureSet_Negative = zeros(1,n_Pressure);
OpeningSet_Negative = zeros(n_Opening,n_Pressure);
FlowRateSet_Negative = zeros(n_Opening,n_Pressure);

for i = 1:n_Pressure
    PressureSet_Negative(i) = sscanf(FolderLists_Pressure(i).name, strcat('Ps_','%d','bar'));
    Path_Pressure = FolderLists_Pressure(i).name;
    Path_Pressure = strcat(Path_Negative,'/',Path_Pressure);
    addpath(Path_Pressure);
    
    FolderLists_Opening = dir(strcat(Path_Pressure,'/OPEN*'));
    n_Opening = length(FolderLists_Opening);
    
    for j = 1:n_Opening
        OpeningName = FolderLists_Opening(j).name;
        Data_temp = load(strcat(Path_Pressure,'/',OpeningName));
        dt = Data_temp(1,1);
        Data_AL = Data_temp(:,2:end);
        
        [m,n] = size(Data_AL);
        t = (0:dt:((m-1)*dt))';
        
        Angle           = Data_AL(:,1);
        AngVel          = Data_AL(:,2);
        ValveOpen       = Data_AL(:,3);
        ValveOpenRef    = Data_AL(:,4);
        PumpPres        = Data_AL(:,5);
        
        rA = 9595.0;
        FlowRateSet_Negative(j,i) = mean(AngVel)/180.0*pi*rA*1e-6*60;
        OpeningSet_Negative(j,i) = mean(ValveOpen);
    end
end

OpeningSet_total = [OpeningSet_Positive; OpeningSet_Negative];
FlowRateSet_total = [FlowRateSet_Positive; FlowRateSet_Negative];

%%
close all;
Kv = zeros(1,n_Pressure);
FlowRateSet_fit = zeros(size(FlowRateSet_total));
% for i = 1:n_Pressure
    
    x = OpeningSet_total(:,i);
    y = FlowRateSet_total(:,i);
    fun = @(r) sum((y - (r(1)*sqrt(PressureSet_Positive(i)/2.0).*tanh(r(2)*(x-r(3))/1000.0))).^2);
    r0 = [1.0; 0.15; -280];
    r_sol = fmincon(fun,r0,[],[]);
    
    y_fit = (r_sol(1)*sqrt(PressureSet_Positive(i)/2.0).*tanh(r_sol(2)*(x-r_sol(3))/1000.0));
    Kv = r_sol(1)
    a = r_sol(2)
    Xv_off = r_sol(3)

    FlowRateSet_fit(:,i) = y_fit;
    
    figure(i);
    plot(OpeningSet_total(:,i),FlowRateSet_total(:,i),'*'); hold on;
    plot(OpeningSet_total(:,i),FlowRateSet_fit(:,i),'*');
    
    figure(1000);
    plot(OpeningSet_total(:,i),FlowRateSet_total(:,i),'*'); hold on;
    figure(1001);
    plot(OpeningSet_total(:,i),FlowRateSet_fit(:,i),'*'); hold on;
end
% 
% 
% x = PressureSet_Positive;
% y = Kv;
% r = optimvar('r',2);
% fun = (r(1)*sqrt((x-r(2))/2.0));
% obj = sum((y - fun).^2);
% lsqproblem = optimproblem("Objective",obj);
% x0.r = [4.0,10.0];
% [sol,fval] = solve(lsqproblem,x0);
% y_fit = sol.r(1)*sqrt((x-sol.r(2))/2.0);
% % sol.r 
% figure(10000);
% plot(x,y,'*'); hold on;
% plot(x,y_fit,'*'); 
% axis([0 100 0 10])
