clear;
close all;

% This file illustrates how to plot both experimental and theoritical data
% on a Bode plot
% Set up your theoretical system transfer function
sys = tf(1,[1 2 3]); %This is an arbitrary system for illustration

% Input the experimental data
mag_data = [4.7447E-1 3.1020E-001 2.2859E-001 4.7135E-001 5.0730E-001 3.3194E-001 1.3731E-001 ...
            3.7142E-002 9.1752E-003 3.3481E-003 4.8627E-004 1.4707E-004];
phase_data = [-3.9811E-001 -2.2486E+000 -3.9605E+000 -9.0757E+000 -2.0766E+001 -4.4318E+001 -1.0900E+002 ...
              -1.4047E+002 -1.6035E+002 -1.8409E+002 -1.6043E+002 -1.9400E+002];
w_data = [1.0000E-002 6.2293E-002 1.1202E-001 2.4495E-001 5.3564E-001 1.0000E+000 2.1062E+000 ...
          4.6058E+000 1.0000E+001 1.8111E+001 3.9604E+001 8.6603E+001];

%Use the Bode command to calculate the theoretical magnitude, phase of the system
[mag,phase,w]=bode(sys);
bode(sys)

% The magnitude and phase are 3 dimensional arrays from the Bode command
% and must be converted to 1 dimensional arrays using the squeeze command
magp = squeeze(mag);
phasep = squeeze(phase);

% Manually plot the theoretical and experimental data on the bode plot
figure
subplot(2,1,1)
semilogx(w,20*log10(magp),w_data,20*log10(mag_data),'o')
xlabel('Frequency (rad/sec)')
ylabel('Magnitude(db)')
subplot(2,1,2)
semilogx(w,phasep,w_data,phase_data,'o')
xlabel('Frequency (rad/sec)')
ylabel('Phase(deg)')