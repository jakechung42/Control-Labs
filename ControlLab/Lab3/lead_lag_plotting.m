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

%% Input Lead circuit experimental circuit
% Input the experimental data
%read data from .dat file
data = load('Lead_data.dat');
f_data = data(:, 1);
mag_data = data(:, 2);
phase_data = data(:, 3);
%convert frequency to angular frequency
w_data = f_data*2*pi;

%Use the Bode command to calculate the theoretical magnitude, phase of the system
[mag,phase,w]=bode(sys);
% bode(sys)

% The magnitude and phase are 3 dimensional arrays from the Bode command
% and must be converted to 1 dimensional arrays using the squeeze command
magp = squeeze(mag);
phasep = squeeze(phase);
% Manually plot the theoretical and experimental data on the bode plot
figure
subplot(2,1,1)
semilogx(w,20*log10(magp),w_data,20*log10(mag_data),'o')
xlabel('Frequency (rad/sec)')
ylabel('Magnitude(db)')
xlim([10 10^4])
title('PD circuit')
subplot(2,1,2)
semilogx(w,phasep,w_data,phase_data,'o')
xlabel('Frequency (rad/sec)')
ylabel('Phase(deg)')
xlim([10 10^4])

%% Set up the theoretical system transfer function for Lead circuit
R1 = 8.192e3;
R2 = 22.078e3;
R3 = 17.966e3;
R4 = 22.008e3;
C1 = 68e-9;
C2 = 0.22e-6;
num = [R1*R2*R4*C1 R2*R4];
den = [R1*R2*R3*C2 R1*R3];
sys = tf(num, den)

%% Input Lag circuit experimental circuit
% Input the experimental data
%read data from .dat file
data = load('Lag_data.dat');
f_data = data(:, 1);
mag_data = data(:, 2);
phase_data = data(:, 3);
%convert frequency to angular frequency
w_data = f_data*2*pi;

%Use the Bode command to calculate the theoretical magnitude, phase of the system
[mag,phase,w]=bode(sys);
% bode(sys)

% The magnitude and phase are 3 dimensional arrays from the Bode command
% and must be converted to 1 dimensional arrays using the squeeze command
magp = squeeze(mag);
phasep = squeeze(phase);
% Manually plot the theoretical and experimental data on the bode plot
figure
subplot(2,1,1)
semilogx(w,20*log10(magp),w_data,20*log10(mag_data),'o')
xlabel('Frequency (rad/sec)')
ylabel('Magnitude(db)')
xlim([10 10^4])
title('PD circuit')
subplot(2,1,2)
semilogx(w,phasep,w_data,phase_data,'o')
xlabel('Frequency (rad/sec)')
ylabel('Phase(deg)')
xlim([10 10^4])
ylim([-60 0])