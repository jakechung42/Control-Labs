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

%% digital system and design in the w domain for PI controller
sysD_ol = c2d(ol_sys, Ts, 'ZOH');
sysD_ol_w = d2c(sysD_ol, 'tustin');
margin(sysD_ol_w)

%specify the PI controller
wg = 3.76;
Gp_wg = 14.1;
Kp = 10^(-Gp_wg/20);
Ki = Kp*wg/10;
w = tf('s');
Gc_PI_w = (Ki+Kp*w)/w;
figure
margin(Gc_PI_w*sysD_ol_w)  

%convert to digital and check the response
Gc_PI_z = c2d(Gc_PI_w, Ts, 'tustin')
sysD_cl_PI = feedback(Gc_PI_z*sysD_ol, 1);
figure
step(sysD_cl_PI)
stepinfo(sysD_cl_PI)

%% design Lead controller to stack with the PI controller
%define system to design lead controller
%convert digital system to w domain 
figure
margin(Gc_PI_w*sysD_ol_w)
phase_increase = 50; %design decision to add 40 phase margin to PI system
a = (1+sind(phase_increase))/(1-sind(phase_increase))
Gp_mag = -10*log10(a)
wm = 21; %obtain this value from margin(Gc_PI_w*sysD_ol_w)
T = 1/(sqrt(a)*wm)
Gc_lead_w = (1+a*T*w)/(1+T*w);
figure
margin(Gc_lead_w*Gc_PI_w*sysD_ol_w)
%convert the lead controller to PI controller
Gc_lead_z = c2d(Gc_lead_w, Ts, 'tustin')
sysD_cl_PI_lead = feedback(Gc_lead_z*Gc_PI_z*sysD_ol, 1)
figure
step(sysD_cl_PI_lead)
stepinfo(sysD_cl_PI_lead)
