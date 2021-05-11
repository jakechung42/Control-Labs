%this code designs the analog PI controller for the control lab
clear
clc
close all

%% specify the system
s = tf('s');
Gp = 2.388E5/(0.002*s^3+3.765*s^2+1621*s+1.194E5);
figure
bode(Gp)
margin(Gp)
grid on
%phase margin is already pretty high
%The system is fairly stable. going to slow it down by using a PI
%controller
Ki = 2.1935;
Kp = 0.547;
G_PI = (Ki+Kp*s)/s;
figure
bode(G_PI)
grid on
figure
bode(G_PI*Gp)
margin(G_PI*Gp)
grid on

%check the step response
control_clrp = feedback(G_PI*Gp, 1);
figure
step(control_clrp)
grid on
stepinfo(control_clrp)

%get data of the bode rp
w = logspace(0.5, 3);
[mag_t, phase_t] = bode(G_PI*Gp, w);
mag_t = squeeze(mag_t);
phase_t = squeeze(phase_t);
f = w/(2*pi);
figure
subplot(2,1,1)
semilogx(f, mag_t)
title('Compensated with PI controller')
ylabel('Magnitude (dB)')
grid on
subplot(2,1,2)
semilogx(f, phase_t)
xlabel('Frequency (Hz)')
ylabel('Phase')
grid on

%% compare step response PI controller theoretical and experimental
path = "D:\Github\Control-Labs\ControlLab\Lab5\PI_Controller_rawData_1.csv";

A = csvread(path, 2, 0);

t_step_raw = A(:,1);
V_in_step_raw = A(:,2);
V_out_step_raw = A(:,3);

%plot raw data
figure
plot(t_step_raw, V_in_step_raw)
hold on
plot(t_step_raw, V_out_step_raw)
%shift the raw data
shift_idx = find(t_step_raw >= -1.544 & t_step_raw <0);
t_step = t_step_raw(shift_idx);
V_in_step = V_in_step_raw(shift_idx);
V_out_step = V_out_step_raw(shift_idx);
t_step = t_step + 1.544; %shift time to 0
V_in_step = V_in_step + abs(min(V_in_step)); %shift to positive 
V_out_step = V_out_step + abs(min(V_out_step)); %shift to positive
%plot the shifted data
figure
plot(t_step, V_in_step, t_step, V_out_step)
xlabel('Time (s)')
ylabel('Voltage (s)')
legend('Input signal', 'Response')
title('Shifted raw data from scope for PI controller implementation')
grid on

%% plot to compare the step response of scope and theoretical
[y, t] = step(control_clrp);
V_out_step_normed = V_out_step/(max(V_in_step_raw)-min(V_in_step_raw)); %normalize the scope data to 1V
figure
plot(t_step, V_out_step_normed, 'd')
hold on
plot(t, y)
title('Comparing the theoretical response and scope data - PI implementation')
legend('Scope', 'Theoretical')
