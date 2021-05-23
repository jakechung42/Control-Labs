%prototype different controllers for the HDD system to test

clc
close all
clear

%% define the ol transfer function
s = tf('s');
ol_sys = 1.437e6/(s^3+484.5*s^2+38025*s+2.53e5)
figure
w = logspace(0, 3);
bode(ol_sys, w)
title('Open-loop HDD system')
grid on

figure
step(ol_sys)
title('Open-loop response')

%% check the close loop proportional controller with K=1
Ts = 1/1e3; %1k Hz samping frequency
ol_sysD = c2d(ol_sys, Ts, 'ZOH') 
cl_sysD_pp = feedback(ol_sysD*1, 1)
fprintf('Closed loop response to proportional controller')
stepinfo(cl_sysD_pp)
figure
step(cl_sysD_pp)

%% Lead control design
K = 1;
sysw = d2c(K*ol_sysD, 'tustin')  %this is actually the w transform
figure
margin(sysw)
w = tf('s');
%control params for lead controller
a = (1+sind(70))/(1-sind(70));
wm = 131;
T = 1/(sqrt(a)*wm);
Gc_lead_w = (1+a*T*w)/(1+T*w);
figure
margin(Gc_lead_w*sysw)

%convert controller to the z domain
Gc_lead_z = c2d(Gc_lead_w, Ts, 'tustin')
%plot the digital response
sysD_cl_lead = feedback(K*ol_sysD*Gc_lead_z, 1)
figure % plot the step response to system with lead controller
step(sysD_cl_lead)
title('Step response to system with Lead controller')
fprintf('Closed loop response to lead controller')
stepinfo(sysD_cl_lead)