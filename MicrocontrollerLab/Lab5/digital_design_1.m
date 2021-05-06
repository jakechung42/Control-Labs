%this code designs the degital system to make the Lead Controller.
clc
clear
close all

%define the motor system from control lab
s = tf('s');
Gp = 2.388E5/(0.002*s^3+3.765*s^2+1621*s+1.194E5);
Ts = 0.001;
sysd_Gp = c2d(Gp, Ts, 'ZOH')

%find the w transform
sysw_Gp = d2c(sysd_Gp, 'tustin')
bode(sysw_Gp)