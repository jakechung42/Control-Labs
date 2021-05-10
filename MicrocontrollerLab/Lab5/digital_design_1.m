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

%from hand calculation in notebook
a = 0.5649;
T = 0.37581;
w = tf('s');
Gc_w = (1+a*T*w)/(1+T*w)
figure
bode(Gc_w*K*sysw)
margin(Gc_w*K*sysw)
grid on

z = tf('z');
temp = (2/Ts)*(z-1)/(z+1);
LagController = (1+0.2123*temp)/(1+0.3758*temp)
sysControlledDigital_cl = feedback(LagController*K*sysD,1)
%step response of compensated system
figure
step(sysControlledDigital_cl)
grid on

%% comparing the theoretical response with the model response controller gain 1
path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab5\CSV_step_response_1.csv";
data = csvread(path, 2, 0);
raw_t_step = data(:,1);
raw_tach_step = data(:,2);
raw_control_signal_step = data(:,3);
% quick plot the raw data
% figure
% plot(raw_t_step, raw_tach_step, '.')
% hold on
% plot(raw_t_step, raw_control_signal_step, '.')
% title('Raw data of step response')
% legend('Tachometer', 'Control signal')
% 
% path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab5\CSV_ramp_response_1.csv";
% data = csvread(path, 2, 0);
% raw_t_ramp = data(:,1);
% raw_tach_ramp = data(:,2);
% raw_control_signal_ramp = data(:,3);
% % quick plot the raw ramp response data
% figure
% plot(raw_t_ramp, raw_tach_ramp, '.')
% hold on
% plot(raw_t_ramp, raw_control_signal_ramp, '.')
% title('Raw data of ramp response')
% legend('Tachometer', 'Control signal')

%% plot the theoretical response and the raw data for step response
%scale and adjust the raw data to show only step response
% temp = raw_t_step - 0.1975; %need to shift the data 
% idx = find(temp>0);
% t_scaled = temp(idx);
% tach_scaled = raw_tach_step(idx);
% tach_scaled = tach_scaled/5; %normalize to the theoretical response
% [y, t] = step(sysD_cl);
% figure
% plot(t_scaled, tach_scaled, '-o')
% hold on
% plot(t, y, 'd')
% grid on
% 
% %plot the difference between raw and theoretical
% t_scaled = t_scaled(1:length(t));
% tach_scaled = tach_scaled(1:length(t));
% figure
% plot(t_scaled, tach_scaled-y, 'd')
% title('Difference between theoretical and experimental')
%% plot the theoretical response and raw data for ramp response
%scale and adjust the raw data to show only ramp response
% idx = find(raw_t_ramp>0 & raw_t_ramp<0.4);
% t_scaled_ramp = raw_t_ramp(idx);
% tach_scaled_ramp = raw_tach_ramp(idx);
% tach_scaled_ramp = tach_scaled_ramp+1;
% t = linspace(0, 0.4, 0.2/Ts);
% ramp_input = t;
% y = lsim(sys_cl, ramp_input, t, 'zoh');
% figure
% plot(t, y, 'd')
% hold on
% plot(t_scaled_ramp, tach_scaled_ramp, 'o') 

%% plot the xmega controller data
path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab5\CSV_Lag_Controller_Xmega_1.csv";
data = csvread(path, 2, 0);
raw_t_lag_xmega = data(:,1);
raw_tach_lag_xmega = data(:,2);
%plot raw data
figure
plot(raw_t_lag_xmega, raw_tach_lag_xmega, 'd')
%shift and normalize the data
idx = find(raw_t_lag_xmega>=0.992 & raw_t_lag_xmega<1.98);
t_lag_xmega = raw_t_lag_xmega(idx);
t_lag_xmega = t_lag_xmega - 0.992;
tach_lag_xmega = raw_tach_lag_xmega(idx);
tach_lag_xmega = tach_lag_xmega/5; %normalize by the input 5V

%% plot the theoretical Lag controller data and experimental lag control data
path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab5\CSV_Lag_Controller_1.csv";
data = csvread(path, 2, 0);
raw_t_lag = data(:,1);
raw_tach_lag = data(:,2);
%plot raw data
figure
plot(raw_t_lag, raw_tach_lag, 'd')
%shift and normalize data
idx = find(raw_t_lag>0.195 & raw_t_lag<0.39);
t_lag = raw_t_lag(idx);
t_lag = t_lag - 0.195;
tach_lag = raw_tach_lag(idx);
tach_lag = tach_lag/5;
[y, t] = step(sysControlledDigital_cl);
figure
plot(t_lag, tach_lag, 'd')
hold on
plot(t, y, 'o')
grid on
title('Comparing response of theoretical and experimental lag controller implementation')
plot(t_lag_xmega, tach_lag_xmega, 'd')
