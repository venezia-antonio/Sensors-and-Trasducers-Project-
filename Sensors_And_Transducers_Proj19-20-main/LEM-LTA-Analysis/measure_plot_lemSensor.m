%% Initialize work-space
clc
clear all
close all

%% Load work-space
load('measurements_lem_sensor.mat');
% Define sampling time for each test
Tw_100 = 0.1; Tw_80 = 0.08; Tw_60 = 0.06; Tw_40 = 0.04;
Tw_30 = 0.03; Tw_25 = 0.025; Tw_20 = 0.02; Tw_15 = 0.015;

% Define time-base for plotting the data
timebase_100 = 0 : Tw_100 : Tw_100*(length(data_vector_100)-1);
timebase_80 = 0 : Tw_80 : Tw_80*(length(data_vector_80)-1);
timebase_60 = 0 : Tw_60 : Tw_60*(length(data_vector_60)-1);
timebase_40 = 0 : Tw_40 : Tw_40*(length(data_vector_40)-1);
timebase_30 = 0 : Tw_30 : Tw_30*(length(data_vector_30)-1);
timebase_25 = 0 : Tw_25 : Tw_25*(length(data_vector_25)-1);
timebase_20 = 0 : Tw_20 : Tw_20*(length(data_vector_20)-1);
timebase_15 = 0 : Tw_15 : Tw_15*(length(data_vector_15)-1);
%% Measure Analysis
std_100 = std(data_vector_100);
std_80 = std(data_vector_80);
std_60 = std(data_vector_60);
std_40 = std(data_vector_40);
std_20 = std(data_vector_20);
std_15 = std(data_vector_20);
std_25 = std(data_vector_25);
std_30 = std(data_vector_30);
%% Sampling time multiple of 20 ms
figure(1)
sgtitle('Time window multiple of ripple current period')
subplot(321)
title('Tw: 20ms')
grid on
hold on
plot(timebase_20,data_vector_20); xlabel('Time[s]');ylabel('Current[mA]');
subplot(322)
title('Tw: 40ms')
grid on
hold on
plot(timebase_40,data_vector_40); xlabel('Time[s]');ylabel('Current[mA]');
subplot(323)
title('Tw: 60ms')
grid on
hold on
plot(timebase_60,data_vector_60); xlabel('Time[s]');ylabel('Current[mA]');
subplot(324)
title('Tw: 80ms')
grid on
hold on
plot(timebase_80,data_vector_80); xlabel('Time[s]');ylabel('Current[mA]');
subplot(325)
title('Tw: 100ms')
grid on
hold on
plot(timebase_100,data_vector_100); xlabel('Time[s]');ylabel('Current[A]');
subplot(326)
x = [20 40 60 80 100];
temp_high = [std_20 std_40 std_60 std_80 std_100]; 
w1 = 0.5; 
bar(x,temp_high,w1,'FaceColor',[0.2 0.2 0.5])
title('STD over sampling time');
xlabel('Sampling Time [ms]');
ylabel('STD');

%% Sampling time non-multiple of 20 ms
figure(2)
sgtitle('Time window non-multiple of ripple current period')
subplot(221)
plot(timebase_15,data_vector_15,'b',timebase_60,data_vector_60,'r');
xlabel('Time[s]');ylabel('Current[mA]');
legend('Tw: 15ms','Tw: 60ms'); grid on; hold on;
subplot(222)
plot(timebase_25,data_vector_25,'b',timebase_60,data_vector_60,'r');
xlabel('Time[s]');ylabel('Current[mA]');
legend('Tw: 25ms','Tw: 60ms'); grid on; hold on;
subplot(223)
plot(timebase_30,data_vector_30,'b',timebase_60,data_vector_60,'r');
xlabel('Time[s]');ylabel('Current[mA]');
legend('Tw: 30ms','Tw: 60ms'); grid on; hold on;
subplot(224)
x = [15 25 30 60];
temp_high = [std_15 std_25 std_30 std_60]; 
w1 = 0.5; 
bar(x,temp_high,w1,'FaceColor',[0.2 0.2 0.5])
title('STD over sampling time');
xlabel('Sampling Time [ms]');
ylabel('STD');
%% Offset Error
figure(3)
plot(timebase_100,offset_error_100,'b');
xlabel('Time[s]'); ylabel('Current[mA]');
grid on;
title('Offset Error');


