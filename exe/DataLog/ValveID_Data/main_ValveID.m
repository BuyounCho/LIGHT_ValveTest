clc; clear all;
close all;

FolderLists = dir('*2021*');
Path_LatestID = FolderLists(end).name;
addpath(Path_LatestID);

FolderLists_Direction = dir(Path_LatestID);
Path_Direction = FolderLists_Direction(end).name;
Path_Direction = strcat(Path_LatestID,'/',Path_Direction);
addpath(Path_Direction);

if (contains(Path_Direction,'Positive'))
    disp('Positive Direction!')
    FolderLists_Pressure = dir(strcat(Path_Direction,'/Ps*'));
    n_Pressure = length(FolderLists_Pressure);
    PressureSet = zeros(n_Pressure,1);
    for i = 1:n_Pressure
        PressureSet(i) = sscanf(FolderLists_Pressure(i).name, strcat('Ps_','%d','bar'));
        Path_Pressure = FolderLists_Pressure(i).name;
        Path_Pressure = strcat(Path_Direction,'/',Path_Pressure);
        addpath(Path_Pressure);
        
        FolderLists_Opening = dir(strcat(Path_Pressure,'/OPEN*'));
        n_Opening = length(FolderLists_Opening);
        OpeningSet = zeros(n_Opening,1);
        FlowRateSet = zeros(n_Opening,1);
        
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
            FlowRateSet(j) = mean(AngVel)/180.0*pi*rA*1e-6*60;
            OpeningSet(j) = mean(ValveOpen);
%             if(OpeningName(6) == 'p')
%                 OpeningSet(j) = str2double(replace(erase(OpeningName,{'OPEN_','.txt'}),'p',''));
%             elseif(OpeningName(6) == 'm')
%                 OpeningSet(j) = str2double(replace(erase(OpeningName,{'OPEN_','.txt'}),'m','-'));
%             end
        end
        figure(1); hold on;
        plot(OpeningSet,FlowRateSet,'*');
    end
    
elseif (contains(Path_Direction,'Negative'))
    disp('Negative Direction!')
    FolderLists_Pressure = dir(strcat(Path_Direction,'/Ps*'));
    n_Pressure = length(FolderLists_Pressure);
    PressureSet = zeros(n_Pressure,1);
    for i = 1:n_Pressure
        PressureSet(i) = sscanf(FolderLists_Pressure(i).name, strcat('Ps_','%d','bar'));
        Path_Pressure = FolderLists_Pressure(i).name;
        Path_Pressure = strcat(Path_Direction,'/',Path_Pressure);
        addpath(Path_Pressure);
        
        FolderLists_Opening = dir(strcat(Path_Pressure,'/OPEN*'));
        n_Opening = length(FolderLists_Opening);
        OpeningSet = zeros(n_Opening,1);
        FlowRateSet = zeros(n_Opening,1);
        
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
            FlowRateSet(j) = mean(AngVel)/180.0*pi*rA*1e-6*60;
            OpeningSet(j) = mean(ValveOpen);
            %             if(OpeningName(6) == 'p')
            %                 OpeningSet(j) = str2double(replace(erase(OpeningName,{'OPEN_','.txt'}),'p',''));
            %             elseif(OpeningName(6) == 'm')
            %                 OpeningSet(j) = str2double(replace(erase(OpeningName,{'OPEN_','.txt'}),'m','-'));
            %             end
        end
        figure(1); hold on;
        plot(OpeningSet,FlowRateSet,'*');
    end
else
    disp('Direction Error!')
end

