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
