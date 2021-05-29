%this code make the Bode plot from HDD response data to find the transfer
%function

clear
clc
close all

%% Read the raw data and plot Bode plot
path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab7\raw_bode_KP15_8kSampling.dat";
raw_data = load(path, '-ascii');
freq_raw = raw_data(:,1);
mag_raw = raw_data(:,2);
phase_raw = raw_data(:,3);

%plot the raw data
figure
subplot(2,1,1)
semilogx(freq_raw, 20*log10(mag_raw), 'd')
ylabel('Magnitude (dB)')
title('Raw frequency response data of HDD system')
grid on
subplot(2,1,2)
semilogx(freq_raw, phase_raw, 'd')
xlabel('Frequency (Hz)')
ylabel('Phase (degs)')
grid on

%% make the theoretical model to compare with raw data
s = tf('s');
K = 1.03;
zeta = 0.15; 
wn = 211;
%adding additional poles
add_pole = 1/(1/1000*s+1);
% add_pole = 1/(s);
% model = K*wn^2/(s^2+2*zeta*wn*s+wn^2)*add_pole
model = K*wn^2/(s^2+2*zeta*wn*s+wn^2)*add_pole
w = logspace(0, 2.8, 5000);
[mag_m, phase_m] = bode(model, w); %data for the model
mag_m = squeeze(mag_m);
phase_m = squeeze(phase_m);
%plot and compare data
figure
subplot(2,1,1)
semilogx(w/(2*pi), 20*log10(mag_m))
hold on
semilogx(freq_raw, 20*log10(mag_raw), 'd')
ylabel('Magnitude (dB)')
subplot(2,1,2)
semilogx(w/(2*pi), phase_m)
hold on
semilogx(freq_raw, phase_raw, 'd')
xlabel('Frequency (Hz)')
ylabel('Phase (degs)')

%% read raw step data
% path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab7\raw_step_Kp15.csv";
% raw_step = csvread(path, 2, 0);
% time_step_scope = raw_step(:,1);
% V_in_step_scope = raw_step(:,2);
% V_out_step_scope = raw_step(:,3);
% 
% %plot the raw data
% figure
% plot(time_step_scope, V_in_step_scope, time_step_scope, V_out_step_scope, 'd')
% title('Raw step data')
% ylabel('Voltage (V)')
% xlabel('Time (s)')
% grid on
% 
% %scale and normalize and shift the data
% min_vin = min(V_in_step_scope);
% v_in_amp = max(V_in_step_scope)-min(V_in_step_scope);
% idx = find(time_step_scope > 0 & time_step_scope < 0.153); %shift time
% time_step_scope = time_step_scope(idx); 
% V_in_step_scope = V_in_step_scope(idx);
% V_in_step_scope = V_in_step_scope - min_vin;
% V_out_step_scope = V_out_step_scope(idx);
% V_out_step_scope = (V_out_step_scope - 0.392)/v_in_amp;
% t_step_mod = linspace(0, max(time_step_scope), length(time_step_scope));
% step_model = step(model, t_step_mod);
% %plot the shifted data
% figure 
% plot(time_step_scope, V_in_step_scope, time_step_scope, V_out_step_scope, 'd')
% title('Raw step data')
% ylabel('Voltage (V)')
% xlabel('Time (s)')
% grid on
% %plot to compare step response of the model and raw data
% figure
% plot(t_step_mod, step_model)
% hold on
% plot(time_step_scope, V_out_step_scope, 'd')
% grid on
% ylabel('Voltage (V)')
% xlabel('Time (s)')
% title('Comparing step response of HDD system with model')

