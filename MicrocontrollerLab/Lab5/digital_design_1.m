%this code designs the degital system to make the Lead Controller.
clc
clear
close all

%define the motor system from control lab
s = tf('s');
Gp = 2.388E5/(0.002*s^3+3.765*s^2+1621*s+1.194E5);
Ts = 0.001; %1k Hz sampling frequency
sysD = c2d(Gp, Ts, 'ZOH') %convert to digital system 

%find the w transform
K = 1; %let's simplify and go with a K=1 for now
sysw = d2c(K*sysD, 'tustin') %this is actually the w transform.
bode(sysw)
margin(sysw)
grid on

%uncompensated step response
sysD_cl = feedback(K*sysD, 1)
sys_cl = feedback(K*Gp, 1);
%step response of uncompensated system
figure
step(sysD_cl)
grid on

%% comparing the theoretical response with the model response controller gain 1
path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab5\CSV_step_response_1.csv";
data = csvread(path, 2, 0);
raw_t_step = data(:,1);
raw_tach_step = data(:,2);
raw_control_signal_step = data(:,3);
% quick plot the raw data
figure
plot(raw_t_step, raw_tach_step, '.')
hold on
plot(raw_t_step, raw_control_signal_step, '.')
title('Raw data of step response')
legend('Tachometer', 'Control signal')

path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab5\CSV_ramp_response_1.csv";
data = csvread(path, 2, 0);
raw_t_ramp = data(:,1);
raw_tach_ramp = data(:,2);
raw_control_signal_ramp = data(:,3);
% quick plot the raw ramp response data
figure
plot(raw_t_ramp, raw_tach_ramp, '.')
hold on
plot(raw_t_ramp, raw_control_signal_ramp, '.')
title('Raw data of ramp response')
legend('Tachometer', 'Control signal')

%% plot the theoretical response and the raw data for step response
%scale and adjust the raw data to show only step response
temp = raw_t_step - 0.1975; %need to shift the data 
idx = find(temp>0);
t_scaled = temp(idx);
tach_scaled = raw_tach_step(idx);
tach_scaled = tach_scaled/5; %normalize to the theoretical response
[y, t] = step(sysD_cl);
figure
plot(t_scaled, tach_scaled, '-o')
hold on
plot(t, y, 'd')
grid on

%plot the difference between raw and theoretical
t_scaled = t_scaled(1:length(t));
tach_scaled = tach_scaled(1:length(t));
figure
plot(t_scaled, tach_scaled-y, 'd')
title('Difference between theoretical and experimental')
%% plot the theoretical response and raw data for ramp response
%scale and adjust the raw data to show only ramp response
idx = find(raw_t_ramp>0 & raw_t_ramp<0.4);
t_scaled_ramp = raw_t_ramp(idx);
tach_scaled_ramp = raw_tach_ramp(idx);
tach_scaled_ramp = tach_scaled_ramp+1;
t = linspace(0, 0.4, 0.2/Ts);
ramp_input = t;
y = lsim(sys_cl, ramp_input, t, 'zoh');
figure
plot(t, y, 'd')
hold on
plot(t_scaled_ramp, tach_scaled_ramp, 'o') 
