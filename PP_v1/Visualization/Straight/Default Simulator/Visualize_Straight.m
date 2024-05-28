clear;
clc;

RawTable = readtable('Output.csv');

t_s = table2array(RawTable(:,1));
vel_in = table2array(RawTable(:,2));
vel_out = table2array(RawTable(:,3));


%Plot
plot(t_s, vel_out, "LineWidth",3);
hold on;
plot(t_s, vel_in, "LineWidth",2.5);

title('Velocity on the Straight Line');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Actual Velocity', 'Reference Velocity')
xlim([0, 10]);
grid on;
hold off;