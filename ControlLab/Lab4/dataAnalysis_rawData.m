% this code plots the data collected from the motor
clear
clc
close all

%% Input the experimental data
%read data from .dat file
data = load('raw_motor_data.dat');
f_data = data(:, 1);
mag_data = data(:, 2);
phase_data = data(:, 3);
%convert frequency to angular frequency
w_data = f_data*2*pi;

%% Find the transfer function that fit the experimental data
Ktotal = 2;
freqn = 55;
omegan = 2*pi*freqn;
zeta = 2.5;

%define the system
num = Ktotal*omegan^2;
den = [1 2*zeta*omegan omegan^2];

sys = tf(num, den)
w = logspace(-1, 3);
[mag_t, phase_t] = bode(sys, w);
mag_t = squeeze(mag_t);
phase_t = squeeze(phase_t);

%% Transfer function of the motor params from data sheet
L = 0.63e-3; % Inductance (H)
Kt = 1.82e-2; % Torque constant (N-m/A)
R = 0.83; % Resistance (Ohm)
Kbac = 1.82e-2; % Back emf constant (V/(rad/s))
I = 4.2e-6; % Rotor inertia (kg-m2)
b = 2.6e-6; % Viscous Damping (N-m-s)
Ktac = 0.02; % Tachometer constant (V/(rad/s))
Kamp = 2.01; % Amplifier gain
num_motor = Kt*Kamp*Ktac;
den_motor = [I*L (b*L + I*R) (R*b + Kt*Kbac)];
sys_motor = tf(num_motor,den_motor)
[mag_m, phase_m] = bode(sys_motor, w);
mag_m = squeeze(mag_m);
phase_m = squeeze(phase_m);

%% plotting procedure
%plot the raw data
subplot(2, 1, 1)
semilogx(w_data, 20*log10(mag_data), 'o')
hold on
semilogx(w, 20*log10(mag_t), w, 20*log10(mag_m))
subplot(2, 1, 2)
semilogx(w_data, phase_data, 'o')
hold on
semilogx(w, phase_t, w, phase_m)

%% define the motor model from experiment
model = sys;
data = "D:\Github\Control-Labs\ControlLab\Lab4\CSV_2021425103255.csv";
A = csvread(data, 2, 0);

t_step = A(:,1);
V_in_step = A(:,2);
V_out_step = A(:,3);

%normalize the response from oscope
t_step = t_step + 0.4981;
V_out_step = V_out_step + 3.67;
V_in_step = V_in_step + 2;

%remove negative time
pos_idx = find(t_step>0 & t_step<0.2);
t_step = t_step(pos_idx);
V_out_step = V_out_step(pos_idx);
V_in_step = V_in_step(pos_idx);

%make a new t_step_mode vector. 
%The t_step array contains non-monolithic values which causes problem in
%the lsim function
t_step_mod = linspace(0, 0.2, length(t_step));

%theoretical response of the motor model
[motor_respose] = lsim(model, V_in_step, t_step_mod);

%plot raw data
figure
plot(t_step,V_in_step,t_step,V_out_step)
%plot compare responses
figure
plot(t_step, V_out_step, t_step, motor_respose);
