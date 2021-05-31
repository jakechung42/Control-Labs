%this code takes a look at the different gain values that can be applied to the DVD system

clc
clear
close all

%% define the DVD model
s = tf('s');
%===========================================
Ktot = 23;
omegan = 50;
zetta = 0.74;
%===========================================
ol_model = Ktot*omegan^2/(s*(s^2+2*zetta*omegan*s+omegan^2));

figure
step(ol_model)
figure
margin(ol_model)
figure
pzmap(ol_model)

%% proportional controller Kp=3
Kp3 = 3;
cl_model_kp3 = feedback(Kp3*ol_model, 1);
figure %check step response of Kp3 model
step(cl_model_kp3)
title('Step response Kp=3')

%compare with scope data
path = "D:\Github\Control-Labs\ControlLab\Lab7\step_proportional_Kp3.csv";
kp3_info = "Step response for proportional Kp=3"
[time, kp3_input, kp3_output] = read_step_data(path, kp3_info, 1.06);
plot_step_compare(cl_model_kp3, time, kp3_output, kp3_info)

%% proportional controller Kp=2
Kp2 = 2;
cl_model_kp2 = feedback(Kp2*ol_model, 1);
figure %check step response of Kp2 model
step(cl_model_kp2)
title('Step response Kp=2')

%% proportional controller Kp=0.5
Kp05 = 0.5;
cl_model_kp05 = feedback(Kp05*ol_model, 1);
figure %check step response of Kp2 model
step(cl_model_kp05)
title('Step response Kp=0.5')

%compare with scope data
path = "D:\Github\Control-Labs\ControlLab\Lab7\step_proportional_Kp05.csv";
kp05_info = "Step response for proportional Kp=0.5"
[time, kp05_input, kp05_output] = read_step_data(path, kp05_info, 0.76);
plot_step_compare(cl_model_kp05, time, kp05_output, kp05_info)


function [time, s_in, s_out] = read_step_data(path, data_description, y_shift)
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
    idx = find(step_time>0 & step_time<0.229);
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
    time = linspace(min(scope_time), max(scope_time), 10000);
    model_step = step(system, time);
    figure %compare the scope and mode step response
    plot(time, model_step)
    hold on
    plot(scope_time, scope_output)
    ylabel('Voltage')
    xlabel('Time')
    title_str = "Comparing the step response of model and scope " + data_info
    title(title_str)
end