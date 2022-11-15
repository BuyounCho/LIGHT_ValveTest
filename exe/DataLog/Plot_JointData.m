if Flag_JointDataPlot
    
    close all;
    
%    FigurePosition = [100 1000 450 300];

    figure(101);
    plot(t,RHY_pos_ref,t,RHY_pos);
    title('RHY Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;

    figure(102);
    plot(t,RHR_pos_ref,t,RHR_pos);
    title('RHR Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;

    figure(103);
    plot(t,RHP_pos_ref,t,RHP_pos);
    title('RHP Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;

    figure(104);
    plot(t,RKN_pos_ref,t,RKN_pos);
    title('RKN Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;

    figure(105);
    plot(t,RAP_pos_ref,t,RAP_pos);
    title('RAP Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;

    figure(106);
    plot(t,RAR_pos_ref,t,RAR_pos);
    title('RAR Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
    
    figure(107);
    plot(t,LHY_pos_ref,t,LHY_pos);
    title('LHY Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;

    figure(108);
    plot(t,LHR_pos_ref,t,LHR_pos);
    title('LHR Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;

    figure(109);
    plot(t,LHP_pos_ref,t,LHP_pos);
    title('LHP Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;

    figure(110);
    plot(t,LKN_pos_ref,t,LKN_pos);
    title('LKN Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;

    figure(111);
    plot(t,LAP_pos_ref,t,LAP_pos);
    title('LAP Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;

    figure(112);
    plot(t,LAR_pos_ref,t,LAR_pos);
    title('LAR Position');
    ylabel('Angle[deg]')
    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%
%    figure(1105);
%    plot(t,[diff(RAP_lin_ref)/dt;0],t,RAP_linvel);
%    title('RAP Linear Velocity');
%    ylabel('Velocity[mm/s]')
%    legend('Ref','Data');
%%    CurrentFigure = gcf;
%%    CurrentFigure.Position = FigurePosition;
%%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(1106);
%    plot(t,[diff(RAR_lin_ref)/dt;0],t,RAR_linvel);
%    title('RAP Linear Velocity');
%    ylabel('Velocity[mm/s]')
%    legend('Ref','Data');
%%    CurrentFigure = gcf;
%%    CurrentFigure.Position = FigurePosition;
%%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(1111);
%    plot(t,[diff(LAP_lin_ref)/dt;0],t,LAP_linvel);
%    title('RAP Linear Velocity');
%    ylabel('Velocity[mm/s]')
%    legend('Ref','Data');
%%    CurrentFigure = gcf;
%%    CurrentFigure.Position = FigurePosition;
%%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(1112);
%    plot(t,[diff(LAR_lin_ref)/dt;0],t,LAR_linvel);
%    title('RAP Linear Velocity');
%    ylabel('Velocity[mm/s]')
%    legend('Ref','Data');
%%    CurrentFigure = gcf;
%%    CurrentFigure.Position = FigurePosition;
%%    FigurePosition(2) = FigurePosition(2) - 50;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%    FigurePosition(1) = FigurePosition(1) + 500;
%    FigurePosition(2) = 1000;
%
%    figure(201);
%    plot(t,RHR_tor_ref,t,RHR_tor);
%    title('RHR Joint Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');    
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(202);
%    plot(t,RHY_tor_ref,t,RHY_tor);
%    title('RHY Joint Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');    
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(203);
%    plot(t,RHP_tor_ref,t,RHP_tor);
%    title('RHP Joint Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(204);
%    plot(t,RKN_tor_ref,t,RKN_tor);
%    title('RKN Joint Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(205);
%    plot(t,RAP_tor_ref,t,RAP_tor);
%    title('RAP Joint Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%%    
%    figure(206);
%    plot(t,RAR_tor_ref,t,RAR_tor);
%    title('RAR Joint Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');    
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%%
%    figure(207);
%    plot(t,LHR_tor_ref,t,LHR_tor);
%    title('LHR Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(208);
%    plot(t,LHY_tor_ref,t,LHY_tor);
%    title('LHY Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(209);
%    plot(t,LHP_tor_ref,t,LHP_tor);
%    title('LHP Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(210);
%    plot(t,LKN_tor_ref,t,LKN_tor);
%    title('LKN Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(211);
%    plot(t,LAP_tor_ref,t,LAP_tor);
%    title('LAP Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(212);
%    plot(t,LAR_tor_ref,t,LAR_tor);
%    title('LAR Torque');
%    ylabel('Torque[Nm]')
%    legend('Ref','Data');
%%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(2205);
%    plot(t,RAP_force_ref);
%    title('RA1 Actuator Force');
%    ylabel('Linear Force[N]')
%    legend('Ref');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(2206);
%    plot(t,RAR_force_ref);
%    title('RA2 Actuator Force');
%    ylabel('Linear Force[N]')
%    legend('Ref');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%        
%    figure(2211);
%    plot(t,LAP_force_ref);
%    title('LA1 Actuator Force');
%    ylabel('Linear Force[N]')
%    legend('Ref');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%    figure(2212);
%    plot(t,LAR_force_ref);
%    title('LA2 Actuator Force');
%    ylabel('Linear Force[N]')
%    legend('Ref');
%    CurrentFigure = gcf;
%    CurrentFigure.Position = FigurePosition;
%    FigurePosition(2) = FigurePosition(2) - 50;
%    
%     %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 
%     FigurePosition(1) = FigurePosition(1) + 500;
%     FigurePosition(2) = 1000;
% 
%     figure(301);
%     plot(t,RHR_vel);
%     title('RHR Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(302);
%     plot(t,RHY_vel);
%     title('RHY Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(303);
%     plot(t,RHP_linvel);
%     title('RHP Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(304);
%     plot(t,RKN_linvel);
%     title('RKN Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(305);
%     plot(t,RAP_linvel);
%     title('RAP Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(306);
%     plot(t,RAR_linvel);
%     title('RAR Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(307);
%     plot(t,LHR_vel);
%     title('LHR Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(308);
%     plot(t,LHY_vel);
%     title('LHY Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(309);
%     plot(t,LHP_linvel);
%     title('LHP Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(310);
%     plot(t,LKN_linvel);
%     title('LKN Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(311);
%     plot(t,LAP_linvel);
%     title('LAP Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%     
%     figure(312);
%     plot(t,LAR_linvel);
%     title('LAR Velocity');
%     ylabel('Velocity[deg/s]')
%     legend('Data');
%     CurrentFigure = gcf;
%     CurrentFigure.Position = FigurePosition;
%     FigurePosition(2) = FigurePosition(2) - 50;
%
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    
%     figure(403);
%     plot(t,RHP_force_ref);
%     title('RHP Linear Force Ref');
%     ylabel('Force[N]')
%     
%     figure(404);
%     plot(t,RKN_force_ref);
%     title('RKN Linear Force Ref');
%     ylabel('Force[N]')
%
%


end