%this code compares the PI controller digital with analog

clc
clear 
close all

%% specify the system
s = tf('s');
Gp = 2.388E5/(0.002*s^3+3.765*s^2+1621*s+1.194E5);
%specify the PI controller
Ki = 2.1935;
Kp = 0.547;
G_PI = (Ki+Kp*s)/s;
%check the step response
control_clrp = feedback(G_PI*Gp, 1);
figure
step(control_clrp)
title('Closed-loop step response to system with PI controller')

%% read the raw data
path = "D:\Github\Control-Labs\ControlLab\Lab7\digital_PI_controller.csv";
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
title('Raw digital PI controller response')
ylabel('Voltage')
xlabel('Time')

%% shift and scale data
scope_input = scope_input - min(scope_input);
scope_output = scope_output - min(scope_output);
idx = find(scope_time > 0 & scope_time < 0.149);
scope_time = scope_time(idx);
scope_input = scope_input(idx);
scope_output = scope_output(idx);
scope_output = scope_output / scope_input_amp;

figure %plot the shifted raw data
plot(scope_time, scope_output)
title('Shifted digital proportional controller response')
ylabel('Voltage')
xlabel('Time')

%% compare step response PI controller theoretical and experimental
path = "D:\Github\Control-Labs\ControlLab\Lab5\PI_Controller_rawData_1.csv";

A = csvread(path, 2, 0);

t_step_raw = A(:,1);
V_in_step_raw = A(:,2);
V_out_step_raw = A(:,3);

V_in_step_amp = max(V_in_step_raw) - min(V_in_step_raw);

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
V_out_step = V_out_step/V_in_step_amp;
%plot the shifted data
figure
plot(t_step, V_in_step, t_step, V_out_step)
xlabel('Time (s)')
ylabel('Voltage (s)')
legend('Input signal', 'Response')
title('Shifted raw data from scope for PI controller implementation')
grid on

%% compare the digital response and modeled response
[model_response, model_time] = step(control_clrp);
figure %compare the raw data and modeled response
plot(model_time, model_response)
hold on
plot(scope_time, scope_output, 'd')
plot(t_step, V_out_step, 'd')
title('Comparing the analog model response with digital controller data')
ylabel('Voltage')
xlabel('Time')
% axis([0, 0.25, 0, 0.8])