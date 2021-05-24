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

%% digital system and design in the w domain for PI controller
sysD_ol = c2d(ol_sys, Ts, 'ZOH');
sysD_ol_w = d2c(sysD_ol, 'tustin');
figure
margin(sysD_ol_w)
w = tf('s');
wg = 3.76;
Gp_wg = 14.1;
Kp = 10^(-Gp_wg/20);
Ki = Kp*wg/10;
Gc_PI_w = Kp+Ki/w;
figure
margin(Gc_PI_w*sysD_ol_w)
Gc_PI_z = c2d(Gc_PI_w, Ts, 'tustin')

%check step response
sysD_cl_PI = feedback(Gc_PI_z*sysD_ol, 1);
step(sysD_cl_PI)
stepinfo(sysD_cl_PI)

%% compare raw data with PI
path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab7\PI_controller_scope_data.csv";
scope_raw = csvread(path, 2, 0);
scope_time = scope_raw(:,1);
scope_input = scope_raw(:,2);
scope_output = scope_raw(:,3);

%plot the raw data
figure
plot(scope_time, scope_input)
hold on
plot(scope_time, scope_output)
grid on
title('Raw scope data for the PI controller')
ylabel('Voltage (V)')
xlabel('Time (s)')

%shift and scale data
scope_input = scope_input - min(scope_input);
idx = find(scope_time >= 0 & scope_time < 0.238);
scope_time = scope_time(idx);
scope_input = scope_input(idx);
scope_output = scope_output(idx);
scope_output = scope_output - 0.432;
scope_output = scope_output / (max(scope_input)-min(scope_input));
figure
plot(scope_time, scope_input)
hold on
plot(scope_time, scope_output)
grid on
title('Shifted PI controller data')
xlabel('Time (s)')
ylabel('Voltage (V)')

[model_data, time_model_data] = step(sysD_cl_PI);
figure
plot(time_model_data, model_data)
hold on
plot(scope_time, scope_output, 'd')
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Comparing the scope data and model data')
axis([0, 0.25, 0, 0.6])