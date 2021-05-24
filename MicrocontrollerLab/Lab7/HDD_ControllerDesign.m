%prototype different controllers for the HDD system to test

clc
close all
clear

%% define the ol transfer function
s = tf('s');
ol_sys = 1.437e6/(s^3+484.5*s^2+38025*s+2.53e5)
figure
w = logspace(0, 3);
bode(ol_sys, w)
title('Open-loop HDD system')
grid on

figure
step(ol_sys)
title('Open-loop response')

%% check the close loop proportional controller with K=1
Ts = 1/5e3; %5k Hz samping frequency
ol_sysD = c2d(ol_sys, Ts, 'ZOH') 
cl_sysD_pp = feedback(ol_sysD*1, 1)
fprintf('Closed loop response to proportional controller')
stepinfo(cl_sysD_pp)
figure
step(cl_sysD_pp)

%% Lead controller design
K = 2; %for running Lead
sysw = d2c(K*ol_sysD, 'tustin')  %this is actually the w transform
figure
margin(sysw)
w = tf('s');
%control params for lead controller
a = (1+sind(80))/(1-sind(80));
Gp_wm = -10*log10(a)
wm = 185; %for Lead and PI controller
% wm = 380; %for Lead and Lag controller
T = 1/(sqrt(a)*wm);
Gc_lead_w = (1+a*T*w)/(1+T*w);
figure
margin(Gc_lead_w*sysw)

%convert controller to the z domain
Gc_lead_z = c2d(Gc_lead_w, Ts, 'tustin')
%plot the digital response
sysD_cl_lead = feedback(K*ol_sysD*Gc_lead_z, 1)
figure % plot the step response to system with lead controller
step(sysD_cl_lead)
title('Step response to system with Lead controller')
fprintf('Closed loop response to lead controller')
stepinfo(sysD_cl_lead)

%% read the raw data
path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab7\Lead_controller_scope_data_2.csv";
raw_scope_data = csvread(path, 2, 0);
scope_time = raw_scope_data(:,1);
scope_input = raw_scope_data(:,2);
scope_output = raw_scope_data(:,3);

%scle and shift the data
scope_input = scope_input - min(scope_input);
scope_output = scope_output - min(scope_output);
scope_output = scope_output / (max(scope_input) - min(scope_input));
%plot raw data
figure
plot(scope_time, scope_input)
hold on
plot(scope_time, scope_output)
title('Raw scope data')
xlabel('Time (s)')
ylabel('Voltage(V)')

idx = find(scope_time >= 0 & scope_time <0.182);
scope_time = scope_time(idx);
scope_input = scope_input(idx);
scope_output = scope_output(idx);
[model_lead_sys, model_lead_time] = step(sysD_cl_lead);

%plot the model vs scope data
figure
plot(model_lead_time, model_lead_sys)
hold on
plot(scope_time, scope_output, 'd')
title('Comparing the step response of system with lead controller')

%% read simulink simulated for the Lead controller
simulink_data_struct = load('LeadStepResponse.mat', '-mat', 'LeadStepResponse');
sl_data = simulink_data_struct.LeadStepResponse;
%plot the raw Simulink data
sl_time = sl_data(:,1);
sl_control_signal = sl_data(:,2);
sl_response = sl_data(:,3);
figure
plot(sl_time, sl_control_signal)
hold on
plot(sl_time, sl_response)
title('Simulink data with saturation')
ylabel('Voltage')
xlabel('Time')
grid on

%plot to compare with scope data
figure
plot(sl_time, sl_response)
hold on
plot(scope_time, scope_output, 'd')
ylabel('Voltage')
xlabel('Time')
axis([0, 0.2, 0, 1])
title('Comparing the scope response with simulink data')