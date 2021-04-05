clear;
clc;
close all;

%% 100Hz first order RC filter
% This file illustrates how to plot both experimental and theoritical data
% on a Bode plot
% Set up your theoretical system transfer function
R = 7.23e3;
C = 0.22e-6;
sys = tf(1,[R*C 1])

% Input the experimental data
%read data from .dat file
data = load('RC100Hz.dat');
f_data = data(:, 1);
mag_data = data(:, 2);
phase_data = data(:, 3);
%convert frequency to angular frequency
w_data = f_data*2*pi;
%offset phase data
phase_data = -phase_data;

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
title('First order RC filter 100Hz')
subplot(2,1,2)
semilogx(w,phasep,w_data,phase_data,'o')
xlabel('Frequency (rad/sec)')
ylabel('Phase(deg)')
xlim([10 10^4])

%% 300Hz first order RC filter
% This file illustrates how to plot both experimental and theoritical data
% on a Bode plot
% Set up your theoretical system transfer function
R = 5.31e3;
C = 0.1e-6;
sys = tf(1,[R*C 1])

% Input the experimental data
%read data from .dat file
data = load('RC300Hz.dat');
f_data = data(:, 1);
mag_data = data(:, 2);
phase_data = data(:, 3);
%convert frequency to angular frequency
w_data = f_data*2*pi;
%offset phase data
phase_data = -phase_data;

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
title('First order RC filter 300Hz')
subplot(2,1,2)
semilogx(w,phasep,w_data,phase_data,'o')
xlabel('Frequency (rad/sec)')
ylabel('Phase(deg)')
xlim([10 10^4])

%% 100Hz second order RC filter
% This file illustrates how to plot both experimental and theoritical data
% on a Bode plot
% Set up your theoretical system transfer function
R = 8.76e3;
C = 68e-9;
sys = tf(1,[(R*C)^2 3*R*C 1])

% Input the experimental data
%read data from .dat file
data = load('RC100Hz_2ndOrder.dat');
f_data = data(:, 1);
mag_data = data(:, 2);
phase_data = data(:, 3);
%convert frequency to angular frequency
w_data = f_data*2*pi;
%offset phase data
phase_data = -phase_data;

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
title('Second order RC filter 100Hz')
subplot(2,1,2)
semilogx(w,phasep,w_data,phase_data,'o')
xlabel('Frequency (rad/sec)')
ylabel('Phase(deg)')
xlim([10 10^4])