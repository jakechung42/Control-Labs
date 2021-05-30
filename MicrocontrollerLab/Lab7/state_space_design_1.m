%This code designs the state space controller for the HDD system

clear
clc
close all

%% define the system
Ts = 1/5e3; %sampling frequency 5k Hz
s = tf('s');
sys_ol = 1.437e6/(s^3+484.5*s^2+38025*s+2.53e5)
sys_ss = ss(sys_ol) %state space representation
[pole_c, zero_c] = pzmap(sys_ss) %continuous poles and zeroes
sysD_ss = c2d(sys_ss, Ts, 'ZOH') %convert to digital state space
[pole_d, zero_d] = pzmap(sysD_ss) %digital poles and zeroes
figure %digital poles and zeros map
pzmap(sysD_ss)
grid on
axis equal

%{
===================
design requirements
===================
%}
T_r = 0.07; %rise time
a = 2.2/T_r;
%==================

%define roots
rootZ1 = exp(Ts*-a)
rootZ2 = exp(Ts*-2*a)
rootZ3 = exp(Ts*-2.5*a);
p_d = [rootZ1, rootZ2, rootZ3];

%calculate the K matrix
K_d = place(sysD_ss.a, sysD_ss.b, p_d)
%calculate the closed-loop matrix A
Acl_d = sysD_ss.a - sysD_ss.b*K_d
eig(Acl_d) %check if the roots are correctly calculated
%rebuild the closed loop system
sysD_ss_cl = ss(Acl_d, sysD_ss.b, sysD_ss.c, 0, Ts)

%check poles and zeros
figure
pzmap(sysD_ss_cl)
grid on
axis equal

%check step response
figure
step(sysD_ss_cl)
stepinfo(sysD_ss_cl)

%check impulse response
figure
impulse(sysD_ss_cl)

%% observer design
%specify the roots
rootZ1_obs = exp(Ts*-a*6);
rootZ2_obs = exp(Ts*-a*6.5);
rootZ3_obs = exp(Ts*-a*7);

%define root matrix p
p_obs = [rootZ1_obs, rootZ2_obs, rootZ3_obs]
L_obs = place(sysD_ss.a', sysD_ss.c', p_obs)'
A1 = sysD_ss.a-sysD_ss.b*K_d-L_obs*sysD_ss.c;