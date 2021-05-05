% plot the 0.2 theoretical and experimental data to compare the built
% circuit and the theoretical circuit

clear
close all
clc

%% Read the raw data
path = "D:\Github\Control-Labs\ControlLab\Lab5\CL_rawData_Bode.dat";
raw_data = load(path, '-ascii');
freq_exp = raw_data(:,1);
mag_exp = raw_data(:,2);
phase_exp = raw_data(:,3);

%% derived bode model
K = 0.2;
num = K*2.388E5;
den = [0.002 3.765 1621 1.194E5+K*2.388E5];
sys = tf(num, den);
Gp = sys;
w = logspace(-1, 3);
[mag_t, phase_t] = bode(Gp, w);
mag_t = squeeze(mag_t);
phase_t = squeeze(phase_t);
f = w/(2*pi);

%% Bode plot the raw data
figure
subplot(2,1,1)
semilogx(freq_exp, 20*log10(mag_exp), 'o')
hold on
semilogx(f, 20*log10(mag_t))
title('Comparing Experimental and Theoretical for Gain circuit 0.2')
ylabel('Gain (dB)')
subplot(2,1,2)
semilogx(freq_exp, phase_exp, 'o')
hold on
semilogx(f, phase_t)
xlabel('Frequency (Hz)')



