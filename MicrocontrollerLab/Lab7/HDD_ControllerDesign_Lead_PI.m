%prototype different controllers for the HDD system to test

clc
close all
clear

%% define the ol transfer function
s = tf('s');
ol_sys = 1.437e6/(s^3+484.5*s^2+38025*s+2.53e5)
w = logspace(0, 3);
% figure
% bode(ol_sys, w)
% title('Open-loop HDD system')
% grid on

% figure
% step(ol_sys)
% title('Open-loop response')

%% check the close loop proportional controller with K=1 (don't need)
Ts = 1/5e3; %5k Hz samping frequency
ol_sysD = c2d(ol_sys, Ts, 'ZOH') 
% fprintf('Closed loop response to proportional controller')
% stepinfo(cl_sysD_pp)
% figure
% step(cl_sysD_pp)

%% Lead controller design
K = 1; %for running Lead and PI controller
sysw = d2c(K*ol_sysD, 'tustin')  %this is actually the w transform
figure
margin(sysw)
w = tf('s');
%control params for lead controller 
%(I need to slow this down and go easy on lead portion other wise it's going to saturate)
a = (1+sind(20))/(1-sind(20));
Gp_wm = -10*log10(a)
wm = 52; %for Lead and PI controller
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

%%compare the step response of Lead controller implementation
path = "D:\Github\Control-Labs\MicrocontrollerLab\Lab7\Lead_step_response.csv";
scope_raw = csvread(path, 2, 0);
scope_time = scope_raw(:,1);
scope_input = scope_raw(:,2);
scope_output = scope_raw(:,3);

%shift the raw data to start from zero 
scope_output = scope_output - min(scope_input) - 0.048; %fine tune the shift with 0.048
scope_input = scope_input - min(scope_input);

%shift to only consider positive time
idx = find(scope_time > 0 & scope_time < 0.1);
scope_time = scope_time(idx);
scope_input = scope_input(idx);
scope_output = scope_output(idx);

figure %plot the raw data
plot(scope_time, scope_input)
hold on
plot(scope_time, scope_output)

scope_time_mod_Lead = linspace(min(scope_time), max(scope_time), length(scope_time));
%don't want to deal with trying to get the sampling time of the scope to match with the sampling time of the model.
%convert the digital model to continuous model
sys_cl_lead = d2c(sysD_cl_lead, 'tustin');
model_lead_response = lsim(sys_cl_lead, scope_input, scope_time_mod_Lead);

figure %compare the scope response and model step response of Lead controller
plot(scope_time_mod_Lead, model_lead_response)
hold on
plot(scope_time, scope_output, '-o')
plot(scope_time, scope_input, '-o')
xlabel('Time')
ylabel('Voltage')
title('Compare step response of Lead controller')

%% design a lag controller design stacking with the lead controller
% Gp_mag = 13.9;
% wg = 41;
% a = 10^(-Gp_mag/20);
% T = 10/(wg*a);
% Gc_lag_w = (1+a*T*w)/(1+T*w)
% %check bode for the lead and lag controller
% figure
% margin(Gc_lag_w*Gc_lead_w*sysw)
% %check the response
% Gc_lag_z = c2d(Gc_lag_w, Ts, 'tustin');
% sysD_cl_lead_lag = feedback(K*Gc_lead_z*Gc_lag_z*ol_sysD, 1);
% step(sysD_cl_lead_lag)
% stepinfo(sysD_cl_lead_lag)
% fprintf('Control equation for Lead Lag controller')
% K*Gc_lead_z*Gc_lag_z

%% design a PI controller to stack with the lead controller the lag controller didn't perform very well
% Gp_mag = 6.72;
% wg = 24;
% Kp = 10^(-Gp_mag/20);
% Ki = Kp*wg/10;
% Gc_PI_w = (Ki+Kp*w)/w;
% figure
% margin(Gc_PI_w*Gc_lead_w*sysw) % bode plot for the Lead PI controller
% % title('Bode plot for Lead PI controller')
% Gc_PI_z = c2d(Gc_PI_w, Ts, 'tustin');
% sysD_cl_lead_PI = feedback(Gc_lead_z*Gc_PI_z*ol_sysD, 1);
% figure
% step(sysD_cl_lead_PI)
% fprintf('Closed loop response lead - PI controller')
% stepinfo(sysD_cl_lead_PI)
% Gc_lead_z*Gc_PI_z
