% This project simulates a quadcopter in outdoor enviroment
%
% Developed by Tzivaras Vasilis
% Contact me at vtzivaras@gmail.com

% Clearing previous simulation variables
clear all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Enviromental and quadcopter constants  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = 9.81;                       % gravity acceleration m/s^2
m = 1;                          % mass in gram
d = 0.25;                       % length of the rods

ct = 3e-6;                      % thrust coefficient N
cq = 1e-7;                      % torque due to drag coefficient N
k = 1;                          % Total force factor

ktx = 0.65;                     % X axis torque factor
kty = 0.05;                     % Y axis torque factor
ktz = 1.8;                      % Z axis toruqe factor
kd = 1;
kthetax=1;                          
ktzd = 1.8;
kvx = 0.2;                        % Velocity constant in X axis
kvy = 1;                        % Velocity constant in Y axis
kvz = 1;                        % Velocity constant in X axis

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% imulation Initializations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = diag([5e-3, 5e-3, 10e-3]);  % Inertia Matrix

startPoint = [0, 0, 0];         % Starting from x, y, z (0, 0, 0)
endPoint = [0, 0, 10];          % Desired final position  x, y, z
theta_z_des = pi/4;

% Simulation start and end time 
startTime = 0;                  % Start time of the simulation (s)
endTime = 20;                   % End time of the simulationh (s)
dt = 0.005;                     % Steps

times = startTime:dt:endTime;   % Vector with all the times
N = numel(times);               % #of times that simulation will run

% Output values, recorded as the simulation runs
x_out = zeros(3,N);             % mass position [X-Y-Z]
v_out = zeros(3,N);             % mass velocity [X-Y-Z]
a_out = zeros(3,N);             % mass acceleration [X-Y-Z]
eng_RPM = zeros(4, N);          % KV of the four motors(INPUT)
omega = zeros(3, N);            % Angular velocities on yaw, pitch and roll
omegadot = zeros(3, N);         % Angular acceleration on yaw, pitch and roll
theta = zeros(3, N);            % Euler angles of yaw, pich, roll
thetadot = zeros(3, N);         % Derivative of yaw, pitch, roll NOT omegadot
thrust = zeros(3, N);           % vector with values only on z axis
torque = zeros (3, N);          % Torque of yaw, pitch, roll
F_des = zeros(1, N);            % Error from the desire position 
theta_des = zeros(3, N);
theta_des = 0;

vel_x_des = zeros(1, N);
theta_x_des = zeros(1, N);
theta_y_des = zeros(1, N);

weight = [0;0;-m*g;];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculating first index values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
F_des(1, 1) = (m*g)+k*(endPoint(3)-x_out(3, 1))+kd*(0-v_out(3, 1));
vector= [F_des(1), torque(1, 1), torque(2, 1), torque(3, 1)]';
gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
eng_RPM(:, 1) = sqrt(gamma\vector);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for index = 2:1:length(times)
    % Thrust controller
    F_des(index) = (m*g)+k*(endPoint(3)-x_out(3, index-1))+kd*(0-v_out(3, index-1));

    % X axis controller
    vel_x_des(index) = kvx*(endPoint(1) - x_out(1, index-1));
    theta_x_des(index) = (m/F_des(index))*kthetax*(vel_x_des(index)-v_out(1, index-1));
    torque(1, index) = 0;%ktx*(theta_x_des(index)-theta(3, index-1))-ktx*thetadot(1, index-1);
    
    % Y axis controller
    velocity_des = 0.5*endPoint(2) - x_out(2, index-1);
    theta_y_des(index) = (m/F_des(index))*0.5*(velocity_des-v_out(2, index-1));
    torque(2, index) = 2;%ktx*(theta_y_des(index)-theta(2, index-1))-ktx*thetadot(2, index-1);
         
    % Z axis controller
    torque(3, index) = 0;%ktz*(theta_z_des-theta(1, index-1))+ktzd*(-v_out(3, index-1));

    vector = [F_des(index), torque(1, index), torque(2, index), torque(3, index)]';
    gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
    eng_RPM(:, index) = sqrt(gamma\vector);

    % Acceleration of the craft
    R = rotation(theta(:, index-1));
    thrust(:, index) = [0; 0; ct * sum(eng_RPM(:, index).^2)];
    weight = [0;0;-m*g;];

    
    a_out(:, index) = (weight + R*thrust(:, index))/m;

    % Veloctiy of the craft
    v_out(:, index) = v_out(:, index-1) + dt * a_out(:, index);

    % Position of the craft
    x_out(:, index) = x_out(:, index-1) + dt * v_out(:, index);
    
    % Calculate omega dot
    omegadot(:, index) = I\(cross(-omega(:, index),I*omega(:, index))+torque(:, index));
    
    % Calculate omega
    omega(:, index) = omega(:, index-1) + dt * omegadot(:, index);
    
    % Calculate thetadot (yaw, pitch, roll derivatives) 
    thetadot(:, index) = omega2thetadot(omega(:, index),theta(:, index-1));
    
    % Calculate theta (yaw, pitch, roll angles) 
    theta(:, index) = theta(:, index-1) + dt * thetadot(:, index);
end

figure('units','normalized','outerposition',[0 0 1 1], 'KeyPressFcn', @visualize);
theta2 = [theta(3, :);theta(2, :);theta(1, :)];
% Put all simulation variables into an output struct.
data = struct('x', x_out, 'theta', theta2, 'vel', v_out, ...
                    'angvel', a_out, 'times', times, 'dt', dt, 'input', eng_RPM, 'F_des', F_des, 'thrust', thrust);

%Create a full screen figure and visualize the simulation
visualize(data);