clear;
clc;

RawTable = readtable('Output.csv');

t_s = table2array(RawTable(:,1));
vel_in = table2array(RawTable(:,2));
vel_out = table2array(RawTable(:,3));

% %Estimate Intial Acceleration
% idx_test_1 = 10;
% idx_test_2 = 50;
% 
% m_acc_1 = (vel_out(idx_test_2) - vel_out(idx_test_1))/(t_s(idx_test_2) - t_s(idx_test_1));
% c_acc_1 = vel_out(idx_test_2) - m_acc_1 * t_s(idx_test_2);
% 
% x_acc = t_s(1:200);
% y_acc = m_acc_1 * x_acc + c_acc_1;
% 
% %Estimate Deceleration
% idx_test2_1 = 300;
% idx_test2_2 = 350;
% 
% m_decc_1 = (vel_out(idx_test2_2) - vel_out(idx_test2_1))/(t_s(idx_test2_2) - t_s(idx_test2_1));
% c_decc_1 = vel_out(idx_test2_2) - m_decc_1 * t_s(idx_test2_2);
% 
% x_decc = t_s(200:500);
% y_decc = m_decc_1 * x_decc + c_decc_1;

%Velocity on the Straight Line
% plot(t_s, vel_out, "LineWidth",3);
% hold on;
% plot(t_s, vel_in)
% plot(x_acc, y_acc);
% ylim([0, 15]);
% 
% title('Velocity on the Straight Line');
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% legend("Velocity Profile", "Set Velocity", "Initial Acceleration Estimate")
% grid on;
% hold off;

%Speed Up Then Brake
plot(t_s, vel_out, "LineWidth",3);
hold on;
plot(t_s, vel_in, "LineWidth",2.5);

title('Velocity on the Straight Line');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Actuall Velocity', 'Reference Velocity')
xlim([0, 15]);
grid on;
hold off;