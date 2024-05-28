RawTable1 = readtable('Output.csv');
RawTable2 = readtable('output3.csv');

test= table2array(RawTable2);
test(any(isnan(test),2),:) = []; 

x_ax = 1:length(test);

t_s = table2array(RawTable1(:,1));
vel_in = table2array(RawTable1(:,2));
vel_out = table2array(RawTable1(:,3));

figure(1)
plot(t_s, vel_out, "LineWidth",3);

figure(2)
plot(x_ax, test(:,2));
hold on;
plot(x_ax, test(:,1));
hold off;