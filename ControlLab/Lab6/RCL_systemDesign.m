%this code characterizes the response of the RLC system and design
%controllers for it.

clear
close all
clc

%% read raw data to make Bode plot
path = "D:\Github\Control-Labs\ControlLab\Lab6\rawData_bode_RLC.dat";
data = load(path);
raw_freq = data(:,1);
raw_ratio = data(:,2);
raw_phase = data(:,3);
raw_phase = -raw_phase;
%plot the raw data
figure
subplot(2,1,1)
semilogx(raw_freq, 20*log10(raw_ratio), 'd')
title('Raw bode plot data for RLC system')
ylabel('Magnitude (dB)')
grid on
subplot(2,1,2)
semilogx(raw_freq, raw_phase, 'd')
xlabel('Frequency (Hz)')
ylabel('Phase (degs)')
grid on

%% find the function fit. 
%define gain, zeta and omega
fn = 1670; %frequency Hz
wn = 2*pi*fn; %rad/s
zeta = 0.121;
K = 0.97;
s = tf('s');
G_fit = K*wn^2/(s^2+2*zeta*wn*s+wn^2);
w = logspace(3.0, 4.5);
f = w/(2*pi);
[mag_fit, phase_fit] = bode(G_fit, w);
mag_fit = squeeze(mag_fit);
phase_fit = squeeze(phase_fit);
figure
subplot(2,1,1)
semilogx(f, 20*log10(mag_fit))
text = ['Compare match with raw bode, wn=', num2str(wn), ', z=', num2str(zeta), ', K=', num2str(K)];
title(text)
ylabel('Magnitude (dB)')
hold on
semilogx(raw_freq, 20*log10(raw_ratio), 'd')
subplot(2,1,2)
semilogx(f, phase_fit)
xlabel('Frequency (Hz)')
ylabel('Phase (degs)')
hold on
semilogx(raw_freq, raw_phase, 'd')
figure %step response of the model
step(G_fit)
Gp = G_fit %Gp is the process to be controlled

%% read raw step response data
path = "D:\Github\Control-Labs\ControlLab\Lab6\rawData_stepResponse.csv";
step_raw_data = csvread(path, 2, 0);
step_time = step_raw_data(:,1);
step_V_in = step_raw_data(:,2);
step_V_out = step_raw_data(:,3);
%plot the raw step data to check
figure
plot(step_time, step_V_in, step_time, step_V_out)
ylabel('Voltage (V)')
xlabel('Time (s)')
title('Raw step data')
%shift the raw data 
idx = find(step_time > 0 & step_time < 0.00504);
step_time = step_time(idx);
step_V_in = step_V_in(idx);
step_V_out = step_V_out(idx);
step_V_out = (step_V_out + 1)/2;
%compare step response 
t_mod = linspace(min(step_time), max(step_time), length(step_time));
[ol_step] = step(Gp, t_mod);
figure
plot(t_mod, ol_step)
hold on
plot(step_time, step_V_out, 'd')
ylabel('Voltage (V)')
xlabel('Time (s)')
title('Comparing the theoretical step response with the experimental step response')

%% design the controller
figure
margin(Gp)
%first try PI controller
Kd = 1/1.5e4;
Gc_PD = 1+Kd*s
figure
margin(Gc_PD*Gp)
sys_cl_PD = feedback(Gc_PD*Gp, 1);
figure
step(sys_cl_PD)
%from hand calculations 
wg = 9.96e3;
gp_wg = 17.5;
Kp = 10^(-gp_wg/20);
Ki = Kp*wg/10;
Gc_PI = (Ki+Kp*s)/s
figure
margin(Gc_PI*Gc_PD*Gp)
sys_cl_PI_PD = feedback(Gc_PI*Gc_PD*Gp, 1);
figure
step(sys_cl_PI_PD)
Gc_PID = Gc_PD*Gc_PI

%% read raw step response data for the PID controller
path = "D:\Github\Control-Labs\ControlLab\Lab6\rawData_PID_controller.csv";
step_PID_raw = csvread(path, 2, 0);
step_time_PID = step_PID_raw(:,1);
step_V_in_PID = step_PID_raw(:,2);
step_V_out_PID = step_PID_raw(:,3);
%plot the raw data
figure
plot(step_time_PID, step_V_in_PID, step_time_PID, step_V_out_PID)
title('Raw data of the PID controled system step response')
ylabel('Voltage (V)')
xlabel('Time (s)')
%scale and shift the time data
idx = find(step_time_PID > 0 & step_time_PID < 0.0446);
step_time_PID = step_time_PID(idx);
step_V_in_PID = step_V_in_PID(idx);
step_V_out_PID = step_V_out_PID(idx);
step_V_out_PID = (step_V_out_PID + 1)/2;
%generate the theoretical response data
time_mod_PID = linspace(min(step_time_PID), max(step_time_PID), length(step_time_PID));
[cl_PID] = step(sys_cl_PI_PD, time_mod_PID);
%plot to compare the data
figure
plot(step_time_PID, step_V_out_PID, 'd')
hold on
plot(time_mod_PID, cl_PID, 'LineWidth', 2)
ylabel('Voltage (V)')
xlabel('Time (s)')
title('Comparing the PID controlled step response')
