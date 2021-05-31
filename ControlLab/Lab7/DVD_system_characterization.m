%this code read the DVD data and characterizes system frequency response 

clc
clear
close all

%% Read in the raw bode data
path = "D:\Github\Control-Labs\ControlLab\Lab7\raw_data_Bode_DVD_Kp_1.dat";
raw_bode = load(path, '-ascii');
f_raw = raw_bode(:,1); %Hz
mag_raw = 20*log10(raw_bode(:,2)); %ratio
phase_raw = raw_bode(:,3); %degs

figure %plot the raw bode data
subplot(2,1,1)
semilogx(f_raw, mag_raw)
ylabel('Magnitude (dB)')
title('Raw bode data DVD system')
grid on
subplot(2,1,2)
semilogx(f_raw, phase_raw)
xlabel('Frequency (Hz)')
ylabel('Phase (deg)')
grid on

%% define the system model to match with data
%===========================================
Ktot = 23
omegan = 50 %rad/s 
zetta = 0.74 %damping
%===========================================

s = tf('s');
sys_model = Ktot*omegan^2/((s^3+2*zetta*omegan*s^2+omegan^2*s+Ktot*omegan^2))

w = logspace(0.4, 2.2);
[model_mag, model_phase] = bode(sys_model, w);

model_mag = 20*log10(squeeze(model_mag));
model_phase = squeeze(model_phase);
f = w/(2*pi);
title_str = "DVD system, Ktot=" + num2str(Ktot) + ", zeta=" + num2str(zetta) + ", omegan=" + num2str(omegan)

figure %plot the raw bode data and the model 
subplot(2,1,1)
semilogx(f_raw, mag_raw, '-o')
hold on
semilogx(f, model_mag)
ylabel('Magnitude (dB)')
title(title_str)
grid on
subplot(2,1,2)
semilogx(f_raw, phase_raw, '-o')
hold on
semilogx(f, model_phase)
xlabel('Frequency (Hz)')
ylabel('Phase (deg)')
grid on

%% make sure to comment out
%define the system model to match with data by looping through values 
% %===========================================
% Ktot = 25
% omegan = 40 %rad/s 
% zetta = 0.4 %damping
% %===========================================
% Ktot_arr = [20 23 27 30];
% omegan_arr = [47 50 53 56];
% zetta_arr = [0.68 0.7 0.72 0.74];

% for aa = 1:length(Ktot_arr)
%     for bb = 1:length(omegan_arr)
%         for cc = 1:length(zetta_arr)
%             Ktot = Ktot_arr(aa);
%             omegan = omegan_arr(bb);
%             zetta = zetta_arr(cc);

%             s = tf('s');
%             sys_model = Ktot*omegan^2/((s^3+2*zetta*omegan*s^2+omegan^2*s+Ktot*omegan^2))

%             w = logspace(0.4, 2.2, 1000);
%             [model_mag, model_phase] = bode(sys_model, w);

%             model_mag = 20*log10(squeeze(model_mag));
%             model_phase = squeeze(model_phase);
%             f = w/(2*pi);
            
%             title_str = "DVD system, Ktot=" + num2str(Ktot) + ", zeta=" + num2str(zetta) + ", omegan=" + num2str(omegan)

%             figure %plot the raw bode data and the model 
%             subplot(2,1,1)
%             semilogx(f_raw, mag_raw, '-o')
%             hold on
%             semilogx(f, model_mag)
%             ylabel('Magnitude (dB)')
%             title(title_str)
%             grid on
%             subplot(2,1,2)
%             semilogx(f_raw, phase_raw, '-o')
%             hold on
%             semilogx(f, model_phase)
%             xlabel('Frequency (Hz)')
%             ylabel('Phase (deg)')
%             grid on
%         end
%     end
% end

%% check step response of model
figure %check step response 
step(sys_model)

path = "D:\Github\Control-Labs\ControlLab\Lab7\step_response_bode_check.csv";
step_raw = csvread(path, 2, 0);
step_time = step_raw(:,1);
step_input = step_raw(:,2);
step_output = step_raw(:,3);

%plot the raw step response to check bode
figure
plot(step_time, step_input)
hold on
plot(step_time, step_output)
title('Raw step response to proportional controller Kp=1')
ylabel('Voltage')
xlabel('Time')

%scale, shift and normalize the raw data
v_in_amp = max(step_input) - min(step_input);
step_output = (step_output + 0.84)/v_in_amp; %shift and normalize
idx = find(step_time>0 & step_time<0.226);
step_time = step_time(idx);
step_output = step_output(idx);
step_input = (step_input(idx)-min(step_input))/v_in_amp; %shift and normalize

figure %plot the shifted and scaled data
plot(step_time, step_output)
hold on
plot(step_time, step_input)
title('Normalized and shifted data')
ylabel('Voltage')
xlabel('Time')

figure %plot the step response of the model
step(sys_model)
[step_model, time_model] = step(sys_model); %step response of the model
figure %plot to compare the step response of model and real system
plot(time_model, step_model)
hold on
plot(step_time, step_output)
grid on
ylabel('Voltage')
xlabel('Time')
title_str = "Comparing step Reponse of model and real system Ktot=" + num2str(Ktot) + ", zeta=" + num2str(zetta) + ", omegan=" + num2str(omegan);
title(title_str)

