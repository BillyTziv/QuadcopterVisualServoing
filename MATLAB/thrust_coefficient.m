% Emax mt2213 brushless motor thrust coefficient estimation algorithm
% Propeller pitch length are 4.5 and 10 respectively

% Clearing previous simulation variables
clear all;
clc;

% Thrust = ct*(omega*omega)
% The following values can be found in the motor specifications table 
T = [130, 220, 290, 370, 430, 480, 540, 590, 640, 670]';
RPM = [2940, 3860, 4400, 4940, 5340, 5720, 5980, 6170, 6410, 6530]';

% 1 RPM equal to 2*pi/60 rad/s
omega = (RPM.*2*3.14/60);
ct = T./omega.^2;

% The best value for ct is the mean of the above array
mean_ct = mean(ct)