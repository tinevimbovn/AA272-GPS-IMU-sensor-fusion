%% Jesica Gonzalez, Karim Kuran, Tinevimbo Ndlovu 
% AA 272 Final Project Code

clc
clear 
close all

%%
% *Reading in Data:*
data = readtable('IMU Raw Data 2.xls');
accel_x = data.LinearAccelerationX_m_s_2_;
accel_y = data.LinearAccelerationY_m_s_2_;
accel_z = data.LinearAccelerationZ_m_s_2_;

% ypos = data.ypos;
time = data.Time_s_;

%%
% *Integrate acceleration to get velocity:*
vel_x = cumtrapz(time, accel_x);
vel_y = cumtrapz(time, accel_y);
vel_z = cumtrapz(time, accel_z);

velres = sqrt(vel_x.^2 + vel_y.^2 + vel_z.^2);

% *Integrate velocity to get position:*
pos_x = cumtrapz(time, vel_x);
pos_y = cumtrapz(time, vel_y);
pos_z = cumtrapz(time, vel_z);

%%
% *Plots:*
lstr = {'FontName','Times New Roman','FontSize',16};
tstr = {'FontName','Times New Roman','FontSize',18};

figure(1);
subplot(3,1,1);
plot(time, pos_x, 'LineWidth',2);
title('X position vs. Time');
xlabel('Time (s)');
ylabel('X Position (m)');

subplot(3,1,2);
plot(time, pos_y, 'LineWidth',2);
title('Y position vs. Time');
xlabel('Time (s)');
ylabel('Y Position (m)');

subplot(3,1,3);
plot(time, pos_z, 'LineWidth',2);
title('Z position vs. Time');
xlabel('Time (s)');
ylabel('Z Position (m)');

resultant = sqrt(pos_x.^2 + pos_y.^2 + pos_z.^2);

figure(2)
plot(time, resultant, 'LineWidth',2);
box on
grid on
grid minor
title('Position vs. Time', tstr{:},'FontWeight','Normal');
xlabel('Time (s)', lstr{:});
ylabel('Position (m)', lstr{:});
set(gca, 'XTickLabel', get(gca, 'XTick'), 'FontName', 'Times New Roman');
set(gca, 'YTickLabel', get(gca, 'YTick'), 'FontName', 'Times New Roman');
% saveas(gcf, 'IMUPlot.jpg');

figure(3) 
plot3(pos_x, pos_y, pos_z, 'LineWidth',2);
box on
grid on
grid minor
title('Position', tstr{:},'FontWeight','Normal');
xlabel('X', lstr{:});
ylabel('Y', lstr{:});
zlabel('Z', lstr{:});
% set(gca, 'XTickLabel', get(gca, 'XTick'), 'FontName', 'Times New Roman');
% set(gca, 'YTickLabel', get(gca, 'YTick'), 'FontName', 'Times New Roman');
% set(gca, 'ZTickLabel', get(gca, 'ZTick'), 'FontName', 'Times New Roman');
% saveas(gcf, '3DIMUPlot.jpg');


% figure(4)
% plot(time, -ypos);