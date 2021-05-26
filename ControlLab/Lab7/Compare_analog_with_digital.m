%compare the analog proportional controller and digital proportional
%controller with the same K gain

clear
clc
close all

%% design the analog system 
K = 1.25;
num = K*2.388E5;
den = [0.002 3.765 1621 1.194E5+K*2.388E5];
sys = tf(num, den);
figure
step(sys)
title('Step response of analog proportional controller')
stepinfo(sys)
grid on

%% plot the raw data
path = 'D:\Github\Control-Labs\ControlLab\Lab7\digital_proportional_gain_1.25_1.csv';
scope_raw = csvread(path, 2, 0);
scope_time = scope_raw(:,1);
scope_input = scope_raw(:,2);
scope_output = scope_raw(:,3);
scope_input_amp = max(scope_input) - min(scope_input);

figure %plot the raw data
plot(scope_time, scope_input)
hold on
plot(scope_time, scope_output)
grid on
title('Raw digital proportional controller response')
ylabel('Voltage')
xlabel('Time')

%% shift and scale data
scope_input = scope_input - min(scope_input);
scope_output = scope_output - min(scope_output);
idx = find(scope_time > 0 & scope_time < 0.1108);
scope_time = scope_time(idx);
scope_input = scope_input(idx);
scope_output = scope_output(idx);
scope_output = scope_output / scope_input_amp;

figure %plot the shifted raw data
plot(scope_time, scope_output)
title('Shifted digital proportional controller response')
ylabel('Voltage')
xlabel('Time')

%% Read raw data of the step response for the 1.25 gain controller
path = "D:\Github\Control-Labs\ControlLab\Lab7\gainController_1.2.csv";
data = csvread(path, 2, 0);
t_step = data(:,1);
V_in_step = data(:,2);
V_out_step = data(:,3);
%plot raw
figure
plot(t_step, V_in_step, t_step, V_out_step)
legend('Input signal', 'Output signal')
xlabel('Time (s)')
ylabel('Voltage (V)')
%shift and scale the data
pos_idx = find(t_step > 0.133 & t_step < 0.251);
t_step = t_step(pos_idx);
t_step = t_step - 0.134;
V_in_step_amp = max(V_in_step) - min(V_in_step);
V_in_step = V_in_step(pos_idx);
V_in_step = V_in_step - min(V_in_step);
V_out_step = V_out_step(pos_idx);
V_out_step = V_out_step - min(V_out_step);
V_out_step = V_out_step/V_in_step_amp;
figure
plot(t_step, V_out_step)

%% compare the digital response and modeled response
model_time = linspace(0, 0.12, 5000);
[model_response] = step(sys, model_time);
figure %compare the raw data and modeled response
plot(model_time, model_response)
hold on
plot(scope_time, scope_output, 'd')
plot(t_step, V_out_step, 'd')
legend('Model response', 'Digital controller output', 'Analog controller output')
title('Comparing the analog model response with digital controller data')
ylabel('Voltage')
xlabel('Time')
grid on