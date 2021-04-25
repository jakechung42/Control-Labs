clear;
close all;
clc;

% file_step = 'D:\Work\Plot_files\NewFile2.csv'
% A = csvread(file_step, 2, 0);
% t_step = A(:,1);
% V_in_step = A(:,2);
% V_out_step = A(:,3);
% figure
% plot(t_step,V_in_step,t_step,V_out_step)
% Shift the input and output up such that they both start at zero
% The lower values are the values of the input and output just before the
% step input starts. These values can be easily obtained from the data file
% V_in_step_lower_value =
% V_out_step_lower_value =
% V_in_step = V_in_step - V_in_step_lower_value;
% V_out_step = V_out_step - V_out_step_lower_value;
% figure
% plot(t_step,V_in_step,t_step,V_out_step)
% Motor Model determined from the Bode analysis
% zeta = % Damping ratio determined from the frequency response
% frequency_n = % Natural Frequency Hz determined from the frequency response
% omega_n = 2*pi*frequency_n; % rad/sec
% Gain = % gain value determined from the frequency response
% num = Gain*omega_n^2;
% den = [1 2*zeta*omega_n omega_n^2];
% sys = tf(num, den);
% [motor_theoretical_reponse] = lsim(sys, V_in_step, t_step);
% figure
% plot(t_step,V_in_step,t_step,V_out_step,t_step,motor_theoretical_reponse)
% figure
% plot(t_step,V_in_step,t_step,V_out_step,t_step,motor_theoretical_reponse)
% xlim([xx xx]) % enter the values here to plot the region you want to see
% legend('Input','Experiment','Theory')
% Motor model
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
bode(sys_motor)
% [motor_parameter_reponse] = lsim(sys_motor, V_in_step, t_step);
% figure
% plot(t_step,V_in_step,t_step,V_out_step,t_step,motor_theoretical_reponse,...
% t_step,motor_parameter_reponse)
% xlim([xx xx]) % enter the values here to plot the region you want to see
% legend('Input','Experiment','Theory','Motor Parameter Model')