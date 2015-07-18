% Emax mt2213 brushless motor torque coefficient estimation algorithm
% Propeller pitch length are 4.5 and 10 respectively

% Clearing previous simulation variables
clear all;
clc;

% torque = watt/omega
% The following values can be found in the motor specifications table 
W = [11, 22, 33, 44, 55, 66, 77, 88, 99, 106]';
RPM = [2940, 3860, 4400, 4940, 5340, 5720, 5980, 6170, 6410, 6530]';

% 1 RPM equal to 2*pi/60 rad/s
omega = (RPM.*2*3.14/60);
cq = W./omega.^3

% The best value for cq is the mean of the above array
mean_cq = mean(cq)