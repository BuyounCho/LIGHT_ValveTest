if Flag_PumpDataPlot

    close all;
fig99 = figure(99); hold on;
grid on;
plot_PsRef = plot(t,PUMP_RefPressure-3.0,'--k','LineWidth',1.5); 
plot_Ps = plot(t,PUMP_Pressure,'b','LineWidth',1.5); 

% Axes properties : https://kr.mathworks.com/help/matlab/ref/matlab.graphics.axis.axes-properties.html
ax_fig99 = fig99.CurrentAxes;
ax_fig99.FontName = 'Arial';
ax_fig99.FontSize = 12;
ax_fig99.Position = [0.150,0.13,0.800,0.80];
ax_fig99.YLim = [20 100];
ax_fig99.Title.String = 'Supply Pressure';
ax_fig99.Title.FontSize = 13;
ax_fig99.XLabel.String = 'Time (sec)';
ax_fig99.XLabel.FontSize = 13;
ax_fig99.YLabel.String = 'Pressure (bar)';
ax_fig99.YLabel.FontSize = 13;

legend( [plot_PsRef,plot_Ps] , {'Reference','Measure'} );
ax_fig99.Legend.NumColumns = 1;
ax_fig99.Legend.Location = 'northeast';
% ax_fig7.Legend.FontSize = 14;

fig98 = figure(98); hold on;
grid on;
plot_WpRef = plot(t,PUMP_RefVelocity,'--k','LineWidth',1.5); 
plot_Wp = plot(t,PUMP_Velocity,'b','LineWidth',1.5); 

% Axes properties : https://kr.mathworks.com/help/matlab/ref/matlab.graphics.axis.axes-properties.html
ax_fig98 = fig98.CurrentAxes;
ax_fig98.FontName = 'Arial';
ax_fig98.FontSize = 12;
ax_fig98.Position = [0.150,0.13,0.800,0.80];
ax_fig98.Title.String = 'Pump Speed';
ax_fig98.Title.FontSize = 13;
ax_fig98.XLabel.String = 'Time (sec)';
ax_fig98.XLabel.FontSize = 13;
ax_fig98.YLabel.String = 'Speed (rpm)';
ax_fig98.YLabel.FontSize = 13;

legend( [plot_WpRef; plot_Wp], {'Reference','Measure'} );
ax_fig98.Legend.NumColumns = 1;
ax_fig98.Legend.Location = 'northeast';

end