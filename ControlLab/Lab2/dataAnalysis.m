clc;
clear;
close all

%quick script to plot the data collected from the Butterworth filter.
%this data is compared with theoretical and the RC filter from last lab
%collected data Butterworth circuit
lab2data_meas = load("Butterworth_Filter_circuit.dat");
f_meas = lab2data_meas(:, 1);
vin_meas = lab2data_meas(:, 2);
vout_meas = lab2data_meas(:, 3);
mag_meas = vout_meas./vin_meas;
mag_meas = 20*log10(mag_meas);

%Butterworth theoretical data
lab2data_theo = load("Butterworth_Filter_circuit_theoretical.dat");
f_theo = lab2data_theo(:, 1);
mag_theo = lab2data_theo(:, 2);

%read RC filter data from Lab 1
dir = "D:\Github\Control-Labs\ControlLab\Lab1\RC300Hz.dat";
lab1data = load(dir);
f_lab1 = lab1data(:, 1);
mag_lab1 = lab1data(:, 2);
mag_lab1 = 20*log10(mag_lab1);

%plotting procedure
semilogx(f_meas, mag_meas, 'o')
hold on
semilogx(f_theo, mag_theo)
semilogx(f_lab1, mag_lab1, 'o')
xlim([10^1, 10^4])
legend('Measured', 'Theoretical', 'Lab1')
xlabel('Frequency (Hz)')
ylabel('Mag (dB)')
grid on