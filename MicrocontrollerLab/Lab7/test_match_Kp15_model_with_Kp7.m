%test to see if the Kp15 model matches with the Kp7 model
clear
clc
close all

%% define system
Kp = 7;
Ts = 1/8e3; %sampling frequency
s = tf('s');
ol_tf = 3.05733e6/(s^3+1.063e3*s^2+1.078e5*s-1.339e6)
% sysD_ol = c2d(ol_tf, Ts, 'ZOH'); %let's compare the digital system
cl_tf = feedback(Kp*ol_tf, 1);

figure
step(cl_tf)
model = cl_tf;

%% read raw step Kp7 data
path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab7\raw_step_Kp7.csv";
raw_step = csvread(path, 2, 0);
time_step_scope = raw_step(:,1);
V_in_step_scope = raw_step(:,2);
V_out_step_scope = raw_step(:,3);

%plot the raw data
figure
plot(time_step_scope, V_in_step_scope, time_step_scope, V_out_step_scope, 'd')
title('Raw step data')
ylabel('Voltage (V)')
xlabel('Time (s)')
grid on

%scale and normalize and shift the data
% min_vin = min(V_in_step_scope);
% v_in_amp = max(V_in_step_scope)-min(V_in_step_scope);
% idx = find(time_step_scope > 0 & time_step_scope < 0.153); %shift time
% time_step_scope = time_step_scope(idx); 
% V_in_step_scope = V_in_step_scope(idx);
% V_in_step_scope = V_in_step_scope - min_vin;
% V_out_step_scope = V_out_step_scope(idx);
% V_out_step_scope = (V_out_step_scope - 0.336)/v_in_amp;
t_step_mod = linspace(min(time_step_scope), max(time_step_scope), length(time_step_scope));
% [step_model, t_step_model] = step(model);
[step_model, t_step_model] = lsim(model, V_in_step_scope, t_step_mod);

%plot the shifted data
figure 
plot(time_step_scope, V_in_step_scope, time_step_scope, V_out_step_scope, '-')
title('Raw step data')
ylabel('Voltage (V)')
xlabel('Time (s)')
grid on
%plot to compare step response of the model and raw data
figure
plot(t_step_model, step_model)
hold on
plot(time_step_scope, V_out_step_scope, '-')
grid on
ylabel('Voltage (V)')
xlabel('Time (s)')
title('Comparing step response of Kp = 15 model with Kp = 7 step response')