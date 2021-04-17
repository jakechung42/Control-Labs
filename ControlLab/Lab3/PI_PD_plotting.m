% This script plots and compare the PI and PD circuit measured data and
% theoretical data
clc
clear
close all
% Set up your theoretical system transfer function
R1 = 5.546e3;
R2 = 11.902e3;
R3 = 14.808e3;
R4 = 14.840e3;
C = 0.215e-6;
num = [0.000000001 R4*R2*C/R3 R4*R2/(R3*R1)];
den = [0.000000001 1];
sys = tf(num, den)

%%
% Input the experimental data
%read data from .dat file
% data = load('RC100Hz.dat');
% f_data = data(:, 1);
% mag_data = data(:, 2);
% phase_data = data(:, 3);
% %convert frequency to angular frequency
% w_data = f_data*2*pi;
% %offset phase data
% phase_data = -phase_data;

%Use the Bode command to calculate the theoretical magnitude, phase of the system
[mag,phase,w]=bode(sys);
% bode(sys)

% The magnitude and phase are 3 dimensional arrays from the Bode command
% and must be converted to 1 dimensional arrays using the squeeze command
magp = squeeze(mag);
phasep = squeeze(phase);
w = w/(2*pi);
% Manually plot the theoretical and experimental data on the bode plot
figure
subplot(2,1,1)
semilogx(w,20*log10(magp))
% semilogx(w,20*log10(magp),w_data,20*log10(mag_data),'o')
% xlabel('Frequency (rad/sec)')
% ylabel('Magnitude(db)')
xlim([10 10^4])
% title('First order RC filter 100Hz')
subplot(2,1,2)
% semilogx(w,phasep,w_data,phase_data,'o')
semilogx(w,phasep)
% xlabel('Frequency (rad/sec)')
% ylabel('Phase(deg)')
xlim([10 10^4])
ylim([-0 90])