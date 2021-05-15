close all
clear all
clc
load('workspace_char_measure.mat')
%%
c_mean = 1e-3.*[c_0 c_280 c_400  c_720 c_1040 c_1390 c_1800 c_2600];
s_mean = [s_0 s_280 s_400  s_720 s_1040 s_1390 s_1800 s_2600];
%%
%from motor datasheet
kphi = 0.3922;
figure(1)
hold on,grid on
title('Static Characteristic @5V ')
[m,q,r] = regrlin(s_mean,c_mean);
y = m*s_mean + q;
speed = 0:1:75;
C = kphi*(m*speed + q);
I = m*speed + q;
h = plot(speed, I , 'r','Linewidth',2);
h1 = plot(speed,C, 'green','Linewidth',2);
ylim([0 2.5]),xlim([0 75])
xlabel('Speed [RPM]');

h2 =plot(s_0*ones(1,length(current_f_0)),1e-3.*current_f_0,'bo')
plot(s_280*ones(1,length(current_f_280)),1e-3.*current_f_280,'bo')
plot(s_400*ones(1,length(current_f_400)),1e-3.*current_f_400,'bo')
plot(s_720*ones(1,length(current_f_720)),1e-3.*current_f_720,'bo')
plot(s_1040*ones(1,length(current_f_1040)),1e-3.*current_f_1040,'bo')
plot(s_1390*ones(1,length(current_f_1390)),1e-3.*current_f_1390,'bo')
plot(s_1800*ones(1,length(current_f_1800)),1e-3.*current_f_1800,'bo')
plot(s_2600*ones(1,length(current_f_2600)),1e-3.*current_f_2600,'bo')
w = speed/9.55;%rad/sec
P = C.*w;
h3 = plot(speed,P,'cyan','Linewidth',2);
%Max efficiency Point
max_eff_ind = find(abs(diff(P)) < 0.001);
max_eff_speed = speed(max_eff_ind);
h4 = plot(max_eff_speed,P(max_eff_ind),'*c','Linewidth',2);
h4.MarkerFaceColor = h4.Color;
legend([h h1 h2 h3 h4],'Current Linear Regression [A]','Torque Linear Regression [Nm]','Measured Current [A]','Mechanical Power [W]','Max efficiency point');


