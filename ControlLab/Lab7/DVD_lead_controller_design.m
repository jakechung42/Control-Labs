%this code designs a lead controller for the DVD system to get fast response

clc
clear
close all

%% define the DVD model
s = tf('s');
%===========================================
Ktot = 23;
omegan = 50;
zetta = 0.74;
%===========================================.
samplingFreq = 1000; %Hz
Ts = 1/samplingFreq;
ol_model = Ktot*omegan^2/(s*(s^2+2*zetta*omegan*s+omegan^2))

%check cl proportional step response continuous system
cl_model_cont = feedback(ol_model, 1);
figure %step response of the continuous proportional controller
step(cl_model_cont)
stepinfo(cl_model_cont)

%digital stuff
K = 1;
ol_model_d = c2d(ol_model, Ts, 'zoh'); %convert to digital system
ol_model_w = d2c(K*ol_model_d, 'tustin');
figure %bode plot in the w domain
margin(ol_model_w) %current phase margin is 50.1 degs

%% design digital lead controller
% go easy on the phase margin to prevent saturation 
%add 20 deg phase margin
w = tf('s');
add_gain = 20; %amount of phase gain to add in degrees
a = (1+sind(add_gain))/(1-sind(add_gain));
Gp_wm = -10*log10(a)
wm = 30; %for Lead controller
T = 1/(sqrt(a)*wm);
Gc_lead_w = (1+a*T*w)/(1+T*w);
figure
margin(Gc_lead_w*ol_model_w)

%convert lead controller to the z domain
Gc_lead_z = c2d(Gc_lead_w, Ts, 'tustin')
%plot the lead digital response
sysD_cl_lead = feedback(K*ol_model_d*Gc_lead_z, 1);
figure %plot the lead controller step response
step(sysD_cl_lead)
stepinfo(sysD_cl_lead)

%% compare theoretical with scope data 
path = "D:\Github\Control-Labs\ControlLab\Lab7\step_lead_controller_DVD.csv";
datainfo = "Step response for lead controller";
[scope_time, scope_in, scope_out] = read_step_data(path, datainfo, 1.02, 0.229);
plot_step_compare(sysD_cl_lead, scope_time, scope_out, datainfo)

%% compare theoretical with scope data slow input frequency
path = "D:\Github\Control-Labs\ControlLab\Lab7\slow_lead_response.csv";
datainfo = "Step response for lead controller - slow input";
[scope_time, scope_in, scope_out] = read_step_data(path, datainfo, 0.464, 0.892);
plot_step_compare(sysD_cl_lead, scope_time, scope_out, datainfo)

%% supporting functions
function [time, s_in, s_out] = read_step_data(path, data_description, y_shift, time_shift)
    %support function to read and output arrays for read csv data
    %path and data_description must be string
    step_raw = csvread(path, 2, 0);
    step_time = step_raw(:,1);
    step_input = step_raw(:,2);
    step_output = step_raw(:,3);

    figure %plot the raw data before scaling and shifting
    plot(step_time, step_input, step_time, step_output)
    title(data_description)
    ylabel('Voltage')
    xlabel('Time')

    %shift and scale the data
    v_in_amp = max(step_input) - min(step_input);
    min_scope_input = min(step_input);
    idx = find(step_time>0 & step_time<time_shift);
    scope_time = step_time(idx);
    scope_input = step_input(idx);
    scope_output = step_output(idx);
    %normalize the data
    scope_input = (scope_input-min_scope_input)/v_in_amp;
    scope_output = (scope_output+y_shift)/v_in_amp;

    %plot the scaled and shifted data
    figure
    plot(scope_time, scope_output, scope_time, scope_input)
    ylabel('Voltage')
    xlabel('Time')
    title_str = "Shifted and normalized " + data_description;
    title(title_str)

    time = scope_time;
    s_in = scope_input;
    s_out = scope_output;
end

function [] = plot_step_compare(system, scope_time, scope_output, data_info)
    %plot to compare step input of system and raw data
    [model_step, time_step] = step(system);
    figure %compare the scope and mode step response
    stairs(time_step, model_step)
    hold on
    plot(scope_time, scope_output)
    ylabel('Voltage')
    xlabel('Time')
    title_str = "Comparing the step response of model and scope " + data_info
    title(title_str)
end
