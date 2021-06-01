% Use this Program simulate the system with friction and then design a
% controller to improve the response

clear;
close all;
clc

Fo = 1;

% System Model parameters determined from the Bode analysis
Zeta         = 0.74;
frequency_n  = 50/(2*pi); % Hz
Gain         = 23;

Omegan      = 2*pi*frequency_n; % rad/sec

Input_Amplitude = .1;

Kinetic_friction_Force = 1000;              % This is the normalized friction force i.e. Friction_force/Inertia
                                            % Adjust this value untill the the theoritical frictional response
                                            % matches the experimental response
                                            
Kinetic_friction_transition_rate = 1000;    % Do not change this value

qd = -.1:.0001:.1;
c4 = Kinetic_friction_Force;
c5 = Kinetic_friction_transition_rate;

friction_force_total = c4*tanh(c5*qd);

figure
plot(qd,friction_force_total)
xlabel('Velocity')
title('Friction Force (Coulomb)')


Ncycles = 5;
Npoints = 1000;

Period = 1; 
tend = Ncycles*Period;
dt   = Period/Npoints;
    
% Calculate the open loop response
Input_Frequency  = 2*pi/Period; %rad/sec for sin input
sim('Motor_Position_Modelw_friction_6_open_loop',[0:dt:tend]);
    
figure
plot(ScopeData3(:,1),ScopeData3(:,2))
hold on
plot(ScopeData4(:,1),ScopeData4(:,2))
% legend('Input','Position')
title('Open Loop Response')

% Closed Loop response
% Proportional Control Illustration

Control_gain = 1;

% The two lines below set up a proportional controller
% but any type of controller can be specified here
Control_num = [Control_gain];     
Control_den = [1];

% Calculate the closed loop response
Input_Frequency  = 2*pi/Period; %rad/sec for sin input
sim('Motor_Position_Modelw_friction_6_closed_loop',[0:dt:tend]);

figure
plot(ScopeData5(:,1),ScopeData5(:,2),ScopeData4(:,1),ScopeData4(:,2))
% legend('Input','Position')
title('Closed Loop Response')



