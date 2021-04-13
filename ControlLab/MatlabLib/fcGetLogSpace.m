function [out] = fcGetLogSpace(fc, n)
%This function takes in fc with n value and spits out a vector containing
%1 decade above and 1 decade below the fc value
format shortG
const = fc*10^-ceil(log10(fc))+1;
power = log10(fc/const);
out = logspace(power-1,power+1,n);
out = out';
end

