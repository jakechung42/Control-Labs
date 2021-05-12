%this code designs the disturbance controller
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
w = logspace(0.5, 3);
f = w/(2*pi);
[mag_clrp, phase_clrp] = bode(control_clrp, w); %bode of the clrp. I should see this from my data collection
mag_clrp = squeeze(mag_clrp);
phase_clrp = squeeze(phase_clrp);
figure
subplot(2, 1, 1)
semilogx(f, 20*log10(mag_clrp))
grid on
title('Bode of the entire system with PI controller')
subplot(2, 1, 2)
semilogx(f, phase_clrp)
grid on
xlabel('Frequency (Hz)')

%% analyze the disturbance response
%check the Bode plot
dis_sys = feedback(Gp, G_PI);
figure
bode(dis_sys)
figure
step(dis_sys)
title('Response of an input step disturbance signal')
stepinfo(dis_sys)


