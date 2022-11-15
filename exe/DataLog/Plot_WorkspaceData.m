if Flag_WorkspaceDataPlot
    close all;
    
%     figure(2001);
%     plot(t,Xref_RF2CoM(:,1),t,Xsen_RF2CoM(:,1));
%     title('RightFoot to CoM');
%     ylabel('X Position[m]')
%     legend('Ref','Sen');
%     
%     figure(2002);
%     plot(t,Xsen_RF2ZMP(:,1));
%     title('RightFoot to ZMP');
%     ylabel('X Position[m]')
%     legend('Sen');
% 
    figure(2003);
    plot(t,Xref_RF2CoM(:,2),t,Xsen_RF2CoM(:,2));
    title('RightFoot to CoM');
    ylabel('Y Position[m]')
    legend('Ref','Sen');
    
%     figure(2004);
%     plot(t,Xsen_RF2ZMP(:,2));
%     title('RightFoot to ZMP');
%     ylabel('Y Position[m]')
%     legend('Sen');
%     
%     input = Xsen_RF2ZMP(:,2);
%     output = Xsen_RF2CoM(:,2);
%     
%     L=length(input);
%     n = 2^nextpow2(L);
%     Fs=500; 
%     f=Fs*(0:(n/2)-1)/n;
%     input_FFT=fft(input,n);
%     output_FFT=fft(output,n);
%     H=output_FFT./input_FFT;
%     HH=H(1:end/2);
%     HMag=abs(HH);
%     HPh=(angle(HH));
%     
%     figure(2005)
%     semilogx(f,20*log10(input_FFT(1:end/2)));
% 
%         
%     figure(2006);
%     ax1=subplot(2,1,1);
%     semilogx(f,20*log10(HMag));
%     grid on
%     ax2=subplot(2,1,2);
%     semilogx(f,HPh*180/pi);
%     grid on
%     linkaxes([ax1 ax2],'x')
%     
%     
%     figure(2002);
%     plot(t,dXsen_RF2CoM(:,1));
%     title('RightFoot to CoM');
%     ylabel('X veclocity[m/s]')
%     legend('Sen');
%     
%         figure(2003);
%     plot(t,dXsen_RF2CoM(:,2));
%     title('RightFoot to CoM');
%     ylabel('X veclocity[m/s]')
%     legend('Sen');
%     
%         figure(2004);
%     plot(t,dXsen_RF2CoM(:,3));
%     title('RightFoot to CoM');
%     ylabel('X veclocity[m/s]')
%     legend('Sen');
    
%     figure(2002);
%     plot(t,Xdes_RF2CoM(:,2),t,Xref_RF2CoM(:,2),t,Xsen_RF2CoM(:,2));
%     title('RightFoot to CoM');
%     ylabel('Y Position[m]')
%     legend('Des','Ref','Sen');
%     
%     figure(2003);
%     plot(t,Xdes_RF2CoM(:,3),t,Xref_RF2CoM(:,3),t,Xsen_RF2CoM(:,3));
%     title('RightFoot to CoM');
%     ylabel('Z Position[m]')
%     legend('Des','Ref','Sen');
%     

    figure(2001);
    plot(t,Xref_Pel2RF(:,1),t,Xsen_Pel2RF(:,1));
    title('Pel to RF X Position');
    ylabel('Position[m]')
    legend('Ref','Sen');
    
    figure(2002);
    plot(t,Xref_Pel2RF(:,2),t,Xsen_Pel2RF(:,2));
    title('Pel to RF Y Position');
    ylabel('Position[m]')
    legend('Ref','Sen');
    
    figure(2003);
    plot(t,Xref_Pel2RF(:,3),t,Xsen_Pel2RF(:,3));
    title('Pel to RF Z Position');
    ylabel('Position[m]')
    legend('Ref','Sen');
    
    figure(2004);
    plot(t,Xref_Pel2LF(:,1),t,Xsen_Pel2LF(:,1));
    title('Pel to LF X Position');
    ylabel('Position[m]')
    legend('Ref','Sen');
    
    figure(2005);
    plot(t,Xref_Pel2LF(:,2),t,Xsen_Pel2LF(:,2));
    title('Pel to LF Y Position');
    ylabel('Position[m]')
    legend('Ref','Sen');
    
    figure(2006);
    plot(t,Xref_Pel2LF(:,3),t,Xsen_Pel2LF(:,3));
    title('Pel to LF Z Position');
    ylabel('Position[m]')
    legend('Ref','Sen');
    
    
    figure(2007);
    plot(t(:,1),Xsen_Pel(:,1));
    title('Pelvis X');
    ylabel('Position[m]')
    legend('Measure');
    
        figure(2008);
    plot(t(:,1),Xsen_Pel(:,2));
    title('Pelvis Y');
    ylabel('Position[m]')
    legend('Measure');
    
        figure(2009);
    plot(t(:,1),Xsen_Pel(:,3));
    title('Pelvis Height');
    ylabel('Position[m]')
    legend('Measure');
    
% 
%     figure(2004);
%     plot(t,dXref_RF2CoM(:,1),t,dXsen_RF2CoM(:,1));
%     title('RF to CoM Y Velocity');
%     ylabel('Velocity[m/s]')
%     legend('Ref','Sen');
%     
%     figure(2011);
%     plot(t,Xref_LF2CoM(:,1),t,Xsen_LF2CoM(:,1));
%     title('LF to CoM X Position');
%     ylabel('Position[m]')
%     legend('Ref','Sen');
%     
%     figure(2012);
%     plot(t,Xref_LF2CoM(:,2),t,Xsen_LF2CoM(:,2));
%     title('LF to CoM Y Position');
%     ylabel('Position[m]')
%     legend('Ref','Sen');
%     
%     figure(2013);
%     plot(t,Xref_LF2CoM(:,3),t,Xsen_LF2CoM(:,3));
%     title('LF to CoM Z Position');
%     ylabel('Position[m]')
%     legend('Ref','Sen');
%     
%     figure(2014);
%     plot(t,dXref_LF2CoM(:,2),t,dXsen_LF2CoM(:,2));
%     title('LF to CoM Y Velocity');
%     ylabel('Velocity[m/s]')
%     legend('Ref','Sen');
% 
%     figure(2021);
%     plot(t,Xref_LF2RF(:,2),t,Xsen_LF2RF(:,2));
%     title('LF to RF Y Position');
%     ylabel('Position[m]')
%     legend('Ref','Sen');


  
end