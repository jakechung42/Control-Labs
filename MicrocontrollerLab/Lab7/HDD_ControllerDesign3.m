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

%% digital system and design in the w domain for PD controller
sysD_ol = c2d(ol_sys, Ts, 'ZOH');
sysD_ol_w = d2c(sysD_ol, 'tustin');
figure
margin(sysD_ol_w)
w = tf('s');
wm = 15; %the gain cross over frequency
Kd = 1/wm;
Gc_PD_w = 1+Kd*w;
figure
margin(Gc_PD_w*sysD_ol_w)
Gc_PD_z = c2d(Gc_PD_w, Ts, 'tustin');

%% design in the w domain for the PI controller
wg = 12; %new gain cross over to get 150 phase margin
Gp_wg = 11.5;
Kp = 10^(-Gp_wg/20);
Ki = Kp*wg/10;
Gc_PI_w = Kp+Ki/w;
%check the PI controller with PD
figure
margin(Gc_PI_w*Gc_PD_w*sysD_ol_w) %this thing is so crap. Margin is not reporting the correct phase margin
%check the response of the PID controller
Gc_PI_z = c2d(Gc_PI_w, Ts, 'tustin')
sysD_cl_PID_z = feedback(Gc_PI_z*Gc_PD_z*sysD_ol, 1);
figure
step(sysD_cl_PID_z)
stepinfo(sysD_cl_PID_z)

