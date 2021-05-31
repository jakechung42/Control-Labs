% Second order model (s^2 + 2*zeta*omegan*s + omegan^2)y = F0

% File: Digital_Control_Design_Simulation_5.m

clear;
close all;

Model_Order = 2;
omegan = 10;
zeta   = 0.1;

A_cont=[0 1;-omegan^2 -2*zeta*omegan];
B_cont=[0;omegan^2];
C_cont=[1 0];
D_cont=0;
Ts = 0.01;

sys_ss = ss(A_cont,B_cont,C_cont,D_cont);       % Form the continuous state space model
[sys_ss_d,G] = c2d(sys_ss,Ts,'zoh');            % Convert the state space mode to digital form
                                                % Note the G term on the output which shows the
                                                % relationship between the digital states
                                                % and the original continuous states (very convenient!)
                                                % See c2d documentation for a full explanation of G. 

% Verify the digital state space model

[step_pos,step_time] = step(sys_ss_d);   % Save values to compare with values calculate using the recursive equation

% Specify the digital State Space Matrices (A, B, C)

A = sys_ss_d.A
B = sys_ss_d.B
D = sys_ss_d.C

n_steps = 1000;
% n_steps = 10000;
time    = (0:1:(n_steps-1))*Ts;

% Model verification
% Calculate the values of the stete variables over the time range using 
% recursive equations (step input)

xk = zeros(Model_Order,1);  % Initial values of the state variables

for k= 1:n_steps
    % Save data for ploting
    x1_save(k) = xk(1);
    x2_save(k) = xk(2);
    
    % Recursive calculation of the open loop response (step input)
    F0   = 1.0;
    xkp1 = A*xk+B*F0;

    % Update the System States
    xk = xkp1;
end

figure
plot(time,x1_save)
title('Position step response - Open Loop Transfer Function - Recursive')
xlabel('Time (sec)')
ylabel('Position (inch)')

figure
plot(time,x2_save)
title('Velocity step response - Open Loop Transfer Function - Recursive')
xlabel('Time (sec)')
ylabel('Velocity (inch/sec)')

figure
plot(step_time,step_pos,time,x1_save)
title('Position step response - Open Loop Transfer Function')
xlabel('Time (sec)')
ylabel('Position (inch)')
legend('Step function','Recursive equation')


% Calculate the values of the stete variables over the time range using 
% recursive equations (sin input)

xk = zeros(Model_Order,1);  % Initial values of the state variables

for k= 1:n_steps
    % Save data for ploting
    x1_save(k) = xk(1);
    x2_save(k) = xk(2);
    
    % Recursive calculation of the open loop response (sin input)
    F0   = sin(6*k*Ts);
    xkp1 = A*xk+B*F0;
    
    % Update the System States
    xk = xkp1;
end

figure
plot(time,x1_save)
title('Position forced (sin) response - Open Loop Transfer Function - Recursive')
xlabel('Time (sec)')
ylabel('Position (inch)')

figure
plot(time,x2_save)
title('Velocity forced response (sin) - Open Loop Transfer Function - Recursive')
xlabel('Time (sec)')
ylabel('Velocity (inch/sec)')


% Design a Standard State Space Controller

% Design the controller

p=[-10 -12];
% Convert specified roots to digital roots

pd=[exp(p(1)*Ts) exp(p(2)*Ts)]; % Digital Controller roots

K=place(A,B,pd)
Acl1=A-B*K
Gcl1_d=ss(Acl1,B,D,0,Ts)

[step_pos_cl,step_time_cl] = step(Gcl1_d,time);   % Save values to compare with values calculate using the recursive equation

eig(Acl1)

% Simulate the response using recursive equations

xk = zeros(Model_Order,1);  % Initial values of the state variables

for k= 1:n_steps
    % Save data for ploting
    x1_save(k) = xk(1);
    x2_save(k) = xk(2);
    
    % Recursive calculation of the closed loop response with state feedback
    F0   = 1.0;
    xkp1 = Acl1*xk+B*F0;
    
    % Update the System States
    xk = xkp1;
end

figure
stairs(step_time_cl,step_pos_cl)
hold on
stairs(time,x1_save)
title({'Closed Loop Position step response','Standard State Space Controller - No Observer (all states perfectly known)'})
xlabel('Time (sec)')
ylabel('Position (inch)')
legend('Step function','Recursive equation')
xlim([0 2]);
ylim([0 1.2]);

% Simulate the response using recursive equations and eliminate matrix multiplications

xk1 = 0;  % Initial values of the state variables
xk2 = 0;  % Initial values of the state variables

for k= 1:n_steps
    % Save data for ploting
    x1_save(k) = xk1;
    x2_save(k) = xk2;
    
    % Recursive calculation of the closed loop response with state feedback
    F0   = 1.0;
    xk1_p1 = Acl1(1,1)*xk1 + Acl1(1,2)*xk2 + B(1)*F0;
    xk2_p1 = Acl1(2,1)*xk1 + Acl1(2,2)*xk2 + B(2)*F0;
    
    % Update the System States
    xk1 = xk1_p1;
    xk2 = xk2_p1;
end

figure
stairs(step_time_cl,step_pos_cl)
hold on
stairs(time,x1_save)
title({'Closed Loop Position step response - No matrix multiplications','Standard State Space Controller - No Observer (all states perfectly known)'})
xlabel('Time (sec)')
ylabel('Position (inch)')
legend('Step function','Recursive equation')
xlim([0 2]);
ylim([0 1.2]);

% Design and add an Observer

p=[-10 -12];    % Controller Roots
% Convert specified roots to digital roots

pd_ob=[exp(5*p(1)*Ts) exp(5*p(2)*Ts)]; % Digital Observer roots 5* Controller roots

L=place(A',D',pd_ob);
L=L'

% Full order Observer

xkm1_simulation        = zeros(Model_Order,1);
xo_km1                 = zeros(Model_Order,1);
Co_km1                 = 0.0;
Rin(1:n_steps)         = 1.0;  %Step input

noise_C_magnitude = 0.005;                                      % Mesurment noise magnitude
noise_x1          = noise_C_magnitude*normrnd(0,1,1,n_steps);   % Noise defined as a normal distribution

% Simulate the closed loop response with an observer using recursive equations

for ii=1:n_steps
    
    % Save data for ploting
    xo_save{ii}              = xo_km1;
    x_simulation_FO_save{ii} = xkm1_simulation;  % Save Simulation SV's

    % Simulation Model (system states estimated from the theoritical model)
    F0            = Rin(ii) - K*xo_km1;  
    xk_simulation = A*xkm1_simulation + B*F0;
    
    % Measurment Simulation (This would be directly measured in the arctual implimentation)
    Co_k = [(xk_simulation(1) + noise_x1(ii))];  % Note Measurment comes from the simulation here

    % Recursive estimation of the closed loop states using a full order observer
    xo_k   = (A - B*K - L*D)*xo_km1 + B*Rin(ii) + L*Co_km1;
    
    % Update the System States
    xo_km1 = xo_k;

    % Update Output
    Co_km1 = Co_k;
    
    % Update simulation
    xkm1_simulation          = xk_simulation;  % Update Simulation Model
    Co_save(ii)              = Co_km1;

end

x_simulation_FO_save_M  = cell2mat(x_simulation_FO_save);

figure
stairs(step_time_cl,step_pos_cl)
hold on
stairs(time,x_simulation_FO_save_M(1,:))
title({'Closed Loop Position step response - No matrix multiplications - Observer','Standard State Space Controller - With Observer'})
xlabel('Time (sec)')
ylabel('Position (inch)')
legend('Step function','Recursive equation - Observer')
xlim([0 2]);
ylim([0 1.2]);

figure
stairs(time,Co_save)
title({'Measurement with noise','Standard State Space Controller - With Observer'})
xlim([0 2]);
ylim([0 1.2]);

% Note when the observer code above is implemented on a microcontroller to
% estimate the systems unmeasured states all of the matrix and vector
% operations will have to be expanded into scalar terms for
% implementation.  In addition, all computations that can be moved outside
% the loop should be moved so these computations are not unnecessarily
% being done each time through the loop and slowing the system down.



% Design a State Space Controller with an outer feedback loop and an integrator

% Design the controller

p=[-10 -12 -14];
% Convert specified roots to digital roots

pd=[exp(p(1)*Ts) exp(p(2)*Ts) exp(p(3)*Ts)]; % Digital Controller roots

% See "Handout 26 Outer Feedback Loop with an Integrator for Digital State Space Control"

Ahat = [A B;zeros(length(A),1)' 0]
Bhat = [zeros(length(A),1);1]
Khat = place(Ahat,Bhat,pd)

[Khat + [zeros(length(A),1)' 1]]
AA = [A-eye(size(A)) B;D*A D*B]

KK = [Khat + [zeros(length(A),1)' 1]]*inv([A-eye(size(A)) B;D*A D*B])
K2 = KK(1:end-1)
K1 = KK(end)
ACL = [A-B*K2 B*K1;-D*A+D*B*K2 1-D*B*K1]
BCL = Bhat
DCL = [D 0]
sys1 = ss(ACL,BCL,DCL,0,Ts)

[step_pos_cli,step_time_cli] = step(sys1,n_steps*Ts);   % Save values to compare with values calculate using the recursive equation

% Simulate the response using recursive equations

xk = zeros(length(DCL),1);  % Initial values of the state variables

for k= 1:n_steps
    % Save data for ploting
    x1_save(k) = xk(1);
    x2_save(k) = xk(2);
    x3_save(k) = xk(3);
    
    % Recursive calculation of the closed loop response with state feedback
    F0   = 1.0;
    xkp1 = ACL*xk+BCL*F0;
    
    % Update the System States
    xk = xkp1;
end

figure
stairs(step_time_cli,step_pos_cli)
hold on
stairs(time,x1_save)
title({'Closed Loop Position step response','Integrator feedback - recursive equations - No Observer (all states perfectly known)'})
xlabel('Time (sec)')
ylabel('Position (inch)')
legend('Step function','Recursive equation')
xlim([0 2]);
ylim([0 1.2]);


% Design and add an Observer

% Convert specified roots to digital roots

% pd_ob=[exp(1*p(1)*Ts) exp(1*p(2)*Ts)];     % Digital Observer roots 1* Controller roots
% pd_ob=[exp(5*p(1)*Ts) exp(5*p(2)*Ts)];     % Digital Observer roots 5* Controller roots
pd_ob=[exp(10*p(1)*Ts) exp(10*p(2)*Ts)];     % Digital Observer roots 10* Controller roots
% pd_ob=[exp(20*p(1)*Ts) exp(20*p(2)*Ts)];   % Digital Observer roots 20* Controller roots
% pd_ob=[exp(100*p(1)*Ts) exp(100*p(2)*Ts)]; % Digital Observer roots 100* Controller roots

L=place(A',D',pd_ob);
L=L'

% Full order Observer

xkm1_simulation        = zeros((Model_Order+1),1);
xo_km1                 = zeros((Model_Order),1);
xoI_km1                = 0.0;
Co_km1                 = 0.0;
Rin(1:n_steps)         = 1.0;  %Step input

noise_C_magnitude = 0.005;                                      % Mesurment noise magnitude
noise_x1          = noise_C_magnitude*normrnd(0,1,1,n_steps);   % Noise defined as a normal distribution

% Simulate the closed loop response with an observer using recursive equations

for ii=1:n_steps
    
    % Save data for ploting
    xo_save{ii}              = xo_km1;
    xoI_save(ii)             = xoI_km1;
    x_simulation_FO_save{ii} = xkm1_simulation;  % Save Simulation SV's

    % Simulation Model (system states estimated from the theoritical model)
    
    F0               =  - K2*[xo_km1(1);xo_km1(2)] + K1*xkm1_simulation(3);  
    zz               = A*[xkm1_simulation(1);xkm1_simulation(2)] + B*F0;
    xk_simulation(1) = zz(1);
    xk_simulation(2) = zz(2);
    xk_simulation(3) = xkm1_simulation(3) + (Rin(ii) - xkm1_simulation(1));      % Integrator feedback term
    
    % Measurment Simulation (This would be directly measured in the arctual implimentation)
    Co_k = [(xk_simulation(1) + noise_x1(ii))];  % Note Measurment comes from the simulation here
    
    % Recursive estimation of the closed loop states using a full order observer
    xo_k   = (A - B*K2 - L*D)*xo_km1 + B*K1*xoI_km1 + L*Co_km1; 
    xoI_k   = xoI_km1 + (Rin(ii) - Co_k);
    
    % Update the System States
    xo_km1  = xo_k;
    xoI_km1 = xoI_k;
    
    % Update Output
    Co_km1 = Co_k;
    
    % Update simulation
    xkm1_simulation          = xk_simulation;  % Update Simulation Model
    Co_save(ii)              = Co_km1;


end

x_simulation_FO_save_M  = cell2mat(x_simulation_FO_save);
xo_save_M               = cell2mat(xo_save);

% Note when the observer code above is implemented on a microcontroller to
% estimate the systems unmeasured states all of the matrix and vector
% operations will have to be expanded into scalar terms for
% implementation.  In addition, all computations that can be moved outside
% the loop should be moved so these computations are not unnecessarily
% being done each time through the loop and slowing the system down.

figure
plot(time,x_simulation_FO_save_M(1,:),time,xo_save_M(1,:))
title('State 1')
xlabel('Time (sec)')
ylabel('Position')
legend('Simulation','Observer')
xlim([0 2]);

figure
plot(time,x_simulation_FO_save_M(2,:),time,xo_save_M(2,:))
title('State 2')
xlabel('Time (sec)')
ylabel('Velocity (inch/sec)')
legend('Simulation','Observer')
xlim([0 2]);

figure
plot(time,x_simulation_FO_save_M(3,:),time,xoI_save(1,:))
title('State 3')
xlabel('Time (sec)')
ylabel('Integrator State')
legend('Simulation','Observer')
xlim([0 2]);

figure
stairs(step_time_cli,step_pos_cli)
hold on
stairs(time,xo_save_M(1,:))
title({'Closed Loop Position step response','Integrator feedback - recursive equations - With Observer'})
xlabel('Time (sec)')
ylabel('Position (inch)')
legend('Step function (No Observer - all states perfectly known)','Recursive equation - With Observer')
xlim([0 2]);
ylim([0 1.2]);





