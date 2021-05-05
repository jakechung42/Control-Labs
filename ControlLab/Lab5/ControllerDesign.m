% Lab 5 controller design implementation
% This is going to be hard. 

clear;
close all;
clc;

%this code design some example controllers for the motor model

%% Read raw data of the step response for the 0.2 gain controller
path = "D:\Github\Control-Labs\ControlLab\Lab5\CSV_20215215582.csv";
data = csvread(path, 2, 0);
figure
t_step = data(:,1);
V_in_step = data(:,2);
V_out_step = data(:,3);
%plot raw
plot(t_step, V_in_step, t_step, V_out_step)
figure
%shift and scale the data
pos_idx = find(t_step > 0 & t_step < 0.08);
t_step = t_step(pos_idx);
V_in_step = V_in_step(pos_idx);
V_in_step = V_in_step + min(V_in_step);
V_out_step = V_out_step(pos_idx);
V_out_step = V_out_step - min(V_out_step);
plot(t_step, V_out_step)

%% Root locus design
num = 2.388E5;
den = [0.002 3.765 1621 1.194E5];
figure
rlocus(num, den)
Gp = tf(num, den);

%open loop response
t_arr = linspace(0, 1, 1000);
step_arr = ones(1, 1000);
figure
step(Gp);

%% Read raw data of the step response for the OLRP
data = "D:\Github\Control-Labs\ControlLab\Lab4\CSV_2021425103255.csv";
A = csvread(data, 2, 0);

t_step_ol = A(:,1);
V_in_step_ol = A(:,2);
V_out_step_ol = A(:,3);

%normalize the response from oscope
t_step_ol = t_step_ol + 0.5;
V_out_step_ol = V_out_step_ol + 3.67;
V_in_step_ol = V_in_step_ol + 2;

%remove negative time
pos_idx = find(t_step_ol>0 & t_step_ol<0.2);
t_step_ol = t_step_ol(pos_idx);
V_out_step_ol = V_out_step_ol(pos_idx);
V_in_step_ol = V_in_step_ol(pos_idx);

%make a new t_step_mode vector. 
%The t_step array contains non-monolithic values which causes problem in
%the lsim function
t_step_mod_ol = linspace(0, 0.2, length(t_step_ol));

%theoretical response of the system model fit
[ol_response] = lsim(Gp, V_in_step_ol, t_step_mod_ol);

%% design params
%want PMO ~ 30%
z_d = 0.75;
t_rise = 0.002;
omegan_d = (1-0.4167*z_d+2.917*z_d^2)/t_rise;
s1 = -z_d*omegan_d+sqrt(1-z_d^2)*omegan_d*j
s2 = -z_d*omegan_d-sqrt(1-z_d^2)*omegan_d*j
%plot the desired roots
% hold on
% plot(-z_d*omegan_d, sqrt(1-z_d^2)*omegan_d, 'd')
% plot(-z_d*omegan_d, -sqrt(1-z_d^2)*omegan_d, 'd')
% hold off
% K = abs(s2^2+1728*s2+1.194e5)/abs(2.388e5)
K = 0.2;
num = K*2.388E5;
den = [0.002 3.765 1621 1.194E5+K*2.388E5];
sys = tf(num, den);
hold on
step(sys);
legend('OLRP', 'CLRP')

%% plot the raw response and the theoretical response
t_step_mod = linspace(0, max(t_step), length(V_out_step));
[step_data] = lsim(sys, V_in_step, t_step_mod);
figure
plot(t_step, V_out_step, 'o')
hold on 
plot(t_step_mod, step_data)

%% plot and compare the response of OLRP and CLRP 0.2 gain controller
figure
plot(t_step, V_out_step, 'o')
hold on 
plot(t_step_mod, step_data)
plot(t_step_ol, V_out_step_ol, 'o')
plot(t_step_ol, V_in_step_ol, 'o')
plot(t_step_mod_ol, ol_response)
plot(t_step, V_in_step, 'o')
legend('CL resp data',...
    'OL resp theoretical',...
    'OL resp data',...
    'OL input',...
    'OL resp theoretical',...
    'Input signal')
xlabel('Time (s)')
ylabel('Voltage (V)')
