% This script plots and compare the Lead and Lag circuit measured data and
% theoretical data
clc
clear
close all
%% Set up the theoretical system transfer function for Lead circuit
R1 = 11.918e3;
R2 = 26.99e3;
R3 = 14.894e3;
R4 = 14.890e3;
C1 = 0.436e-6;
C2 = 98.4e-9;
num = [R1*R2*R4*C1 R2*R4];
den = [R1*R2*R3*C2 R1*R3];
sys = tf(num, den)
bode(sys)