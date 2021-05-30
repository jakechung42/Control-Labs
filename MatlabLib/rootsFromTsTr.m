%function to output roots requirements from input T_s, T_r, and Ts (sampling period)
%T_s: time settle, T_r: time rise.
%Ts: sampling period

function [root1, root2] = rootsFromTsTr(T_r, T_s, Ts)
    syms wn z
    eq1 = T_r  == (0.8 +2.5*z)/wn;
    eq2 = T_s == 3.2/(z*wn);
    [wn, z] = solve(eq1, eq2, wn, z); %solve for natural freq and zeta
    wn = vpa(wn); %evaluate
    wn = double(wn); %convert to double type
    z = vpa(z);  %evaluate
    z = double(z); %convert to double type
    z = z(find(z>0)); %only use positive value of z
    wn = wn(find(wn>0)); %only use positive value of wn
    wd = sqrt(1-z^2)*wn; %rad/s
    ws = 2*pi/Ts; %rad/s
    z_mag = double(exp(-2*pi*z*wd/(sqrt(1-z^2)*ws)));
    z_angle = double(2*pi*wd/ws); %rad
    root1 = (z_mag*cos(z_angle)+z_mag*sin(z_angle)*j);
    root2 = (z_mag*cos(z_angle)-z_mag*sin(z_angle)*j);
end