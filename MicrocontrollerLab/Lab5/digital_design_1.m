%this code designs the degital system to make the Lead Controller.
clc
clear
close all

%define the motor system from control lab
s = tf('s');
Gp = 2.388E5/(0.002*s^3+3.765*s^2+1621*s+1.194E5);
Ts = 0.001; %1k Hz sampling frequency
sysD = c2d(Gp, Ts, 'ZOH') %convert to digital system 

%find the w transform
K = 1; %let's simplify and go with a K=1 for now
sysw = d2c(K*sysD, 'tustin') %this is actually the w transform.
bode(sysw)
margin(sysw)
grid on

%uncompensated step response
sysD_cl = feedback(K*sysD, 1)
%step response of uncompensated system
figure
step(sysD_cl)
grid on