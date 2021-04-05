clear;
clc;
close all;

%compare the 1st order and 2nd order RC filters
%define the 1st order 100Hz filter
R1 = 7.23e3;
C1 = 0.22e-6;
sys1 = tf(1,[R1*C1 1]);

%define the 2st order 100Hz filter
R2 = 8.76e3;
C2 = 68e-9;
sys2 = tf(1,[(R2*C2)^2 3*R2*C2 1]);

%Use the Bode command to calculate the theoretical magnitude, phase of the system
[mag1,phase1,w1]=bode(sys1);
% bode(sys)

% The magnitude and phase are 3 dimensional arrays from the Bode command
% and must be converted to 1 dimensional arrays using the squeeze command
magp1 = squeeze(mag1);
phasep1 = squeeze(phase1);

%Use the Bode command to calculate the theoretical magnitude, phase of the system
[mag2,phase2,w2]=bode(sys2);
% bode(sys)

% The magnitude and phase are 3 dimensional arrays from the Bode command
% and must be converted to 1 dimensional arrays using the squeeze command
magp2 = squeeze(mag2);
phasep2 = squeeze(phase2);

%read data from .dat file for the 100Hz filter
data1 = load('RC100Hz.dat');
f_data1 = data1(:, 1);
mag_data1 = data1(:, 2);
phase_data1 = data1(:, 3);
%convert frequency to angular frequency
w_data1 = f_data1*2*pi;
%offset phase data
phase_data1 = -phase_data1;

%read data from .dat file for the 2nd order 100Hz filter
data2 = load('RC100Hz_2ndOrder.dat');
f_data2 = data2(:, 1);
mag_data2 = data2(:, 2);
phase_data2 = data2(:, 3);
%convert frequency to angular frequency
w_data2 = f_data2*2*pi;
%offset phase data
phase_data2 = -phase_data2;

% Manually plot the theoretical and experimental data on the bode plot
figure
subplot(2,1,1)
semilogx(w1,20*log10(magp1),w_data1,20*log10(mag_data1),'o')
xlabel('Frequency (rad/sec)')
ylabel('Magnitude(db)')
xlim([10 10^4])
hold on
semilogx(w2,20*log10(magp2),w_data2,20*log10(mag_data2),'o')
xlim([10 10^4])
title('Second order RC filter 100Hz')
subplot(2,1,2)
semilogx(w1,phasep1,w_data1,phase_data1,'o')
xlabel('Frequency (rad/sec)')
ylabel('Phase(deg)')
hold on
semilogx(w2,phasep2,w_data2,phase_data2,'o')
xlim([10 10^4])
