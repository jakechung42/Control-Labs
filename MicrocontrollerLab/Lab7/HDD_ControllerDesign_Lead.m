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

%% Lead controller design
K = 1; %for running Lead and PI controller
% K = 3; %for running Lead and Lag controller
sysw = d2c(K*ol_sysD, 'tustin')  %this is actually the w transform
figure
margin(sysw)
w = tf('s');
%control params for lead controller
a = (1+sind(80))/(1-sind(80));
Gp_wm = -10*log10(a)
wm = 185; %for Lead and PI controller
% wm = 380; %for Lead and Lag controller
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


%% design a lag controller design stacking with the lead controller
% Gp_mag = 13.9;
% wg = 41;
% a = 10^(-Gp_mag/20);
% T = 10/(wg*a);
% Gc_lag_w = (1+a*T*w)/(1+T*w)
% %check bode for the lead and lag controller
% figure
% margin(Gc_lag_w*Gc_lead_w*sysw)
% %check the response
% Gc_lag_z = c2d(Gc_lag_w, Ts, 'tustin');
% sysD_cl_lead_lag = feedback(K*Gc_lead_z*Gc_lag_z*ol_sysD, 1);
% step(sysD_cl_lead_lag)
% stepinfo(sysD_cl_lead_lag)
% fprintf('Control equation for Lead Lag controller')
% K*Gc_lead_z*Gc_lag_z

%% design a PI controller to stack with the lead controller the lag controller didn't perform very well
% Gp_mag = 6.72;
% wg = 24;
% Kp = 10^(-Gp_mag/20);
% Ki = Kp*wg/10;
% Gc_PI_w = (Ki+Kp*w)/w;
% figure
% margin(Gc_PI_w*Gc_lead_w*sysw) % bode plot for the Lead PI controller
% % title('Bode plot for Lead PI controller')
% Gc_PI_z = c2d(Gc_PI_w, Ts, 'tustin');
% sysD_cl_lead_PI = feedback(Gc_lead_z*Gc_PI_z*ol_sysD, 1);
% figure
% step(sysD_cl_lead_PI)
% fprintf('Closed loop response lead - PI controller')
% stepinfo(sysD_cl_lead_PI)
% Gc_lead_z*Gc_PI_z
