% Lab 5 controller design implementation
% This is going to be hard. 

clear;
close all;
clc;

%this code design some example controllers for the motor model

%% Root locus design
zeta = 2.5;
omegan = 2*pi*55;
gain = 2;

num = 2.388e5;
den = [1 1728 1.194E5];
rlocus(num, den)
Gp = tf(num, den);

%open loop response
t_arr = linspace(0, 1, 1000);
step_arr = ones(1, 1000);
step(Gp);

%% design params
%want PMO ~ 30%
z_d = 0.75;
t_rise = 0.002;
omegan_d = (1-0.4167*z_d+2.917*z_d^2)/t_rise;
s1 = -z_d*omegan_d+sqrt(1-z_d^2)*omegan_d*j
s2 = -z_d*omegan_d-sqrt(1-z_d^2)*omegan_d*j
%plot the desired roots
hold on
plot(-z_d*omegan_d, sqrt(1-z_d^2)*omegan_d, 'd')
plot(-z_d*omegan_d, -sqrt(1-z_d^2)*omegan_d, 'd')
hold off
% K = abs(s2^2+1728*s2+1.194e5)/abs(2.388e5)
K=0.2;
num = K*2.388E5;
den = [1 1728 1.194E5+K*2.388E5];
sys = tf(num, den);
hold on
step(sys);

