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
endPoint = [2, 3, 10];          % Desired final position  x, y, z
%theta_z_des = 1;
theta_z_des = 45*(pi/180);% kw

% Simulation start and end time 
startTime = 0;                  % Start time of the simulation (s)
endTime = 20;                   % End time of the simulationh (s)
dt = 0.005;                     % Steps
%dt = 0.0001; %kw

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

vel_y_des = zeros(1, N);
vel_x_des = zeros(1, N);
theta_x_des = 0;
theta_y_des = 0;

weight = [0;0;-m*g;];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculating first index values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% F_des(1, 1) = (m*g)+k*(endPoint(3)-x_out(3, 1))+kd*(0-v_out(3, 1));
% vector= [F_des(1), torque(1, 1), torque(2, 1), torque(3, 1)]';
% gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
% eng_RPM(:, 1) = sqrt(gamma\vector);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v = [0 0 0]';% kw
x = startPoint';% kw
omega = [0 0 0]';% kw
theta = [0 0 0]';% kw
for index = 1:1:length(times)
    % Thrust controller
    %F_des(index) = (m*g)+k*(endPoint(3)-x_out(3, index-1))+kd*(0-v_out(3, index-1));
    F_des(index) = (m*g)+k*(endPoint(3)-x(3))+kd*(0-v(3));% kw

    % X axis controller
    vel_x_des = kvx*(endPoint(1) - x(1));
    theta_x_des = (m/F_des(index))*kthetax*(vel_x_des-v(1));
    torque(1, index) = ktx*(theta_x_des - theta(3))-ktx*thetadot(3);
    
    % Y axis controller
    vel_y_des = 0.5*endPoint(2) - x(2);
    theta_y_des = (m/F_des(index))*0.5*(vel_y_des-v(2));
    torque(2, index) = ktx*(theta_y_des-theta(2))-ktx*thetadot(2);
         
    % Z axis controller
    %torque(3, index) = ktz*(theta_z_des-theta(1, index-1))+ktzd*(-v_out(3, index-1));
    torque(3, index) = ktz*(theta_z_des-theta(1))+ktzd*(0-thetadot(1));% kw

    vector = [F_des(index), torque(1, index), torque(2, index), torque(3, index)]';
    gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
    %eng_RPM(:, index) = sqrt(gamma\vector);
    eng_RPM(:, index) = sqrt(abs(gamma\vector));% kw

    % Acceleration of the craft
    R = rotation(theta);
    thrust(:, index) = [0; 0; ct * sum(eng_RPM(:, index).^2)];
    %weight = [0;0;-m*g;];

    
    %a_out(:, index) = (weight + R*thrust(:, index))/m;
    a = (weight + R*thrust(:, index))/m;% kw

    % Veloctiy of the craft
    %v_out(:, index) = v_out(:, index-1) + dt * a_out(:, index);
    v = v + dt * a;% kw

    % Position of the craft
    %x_out(:, index) = x_out(:, index-1) + dt * v_out(:, index);
    x = x + dt * v;% kw
    
  
    % Update the altitude variable for the next loop
    %x_out(:, index) = x_out(:, index);
    x_out(:, index) = x;% kw
    v_out(:, index) = v;% kw
    a_out(:, index) = a;% kw
    
    % Calculate omega dot
    %omegadot(:, index) = I\(cross(-omega(:, index),I*omega(:, index))+torque(:, index));
    omegadot = I\(cross(-omega,(I*omega))+torque(:, index));% kw
    
    % Calculate omega
    %omega(:, index) = omega(:, index-1) + dt * omegadot(:, index);
    omega = omega + dt * omegadot;% kw
    
    % Calculate thetadot (yaw, pitch, roll derivatives) 
    %thetadot(:, index) = omega2thetadot(omega(:, index),theta(:, index-1));
    thetadot = omega2thetadot(omega,theta);% kw
    
    % Calculate theta (yaw, pitch, roll angles) 
    %theta(:, index) = theta(:, index-1) + dt * thetadot(:, index);
    theta = theta + dt * thetadot;% kw
    
    theta_out(:, index) = theta;% kw
end

figure('units','normalized','outerposition',[0 0 1 1], 'KeyPressFcn', @visualize);
% theta2 = [theta(3, :);theta(2, :);theta(1, :)];
% % Put all simulation variables into an output struct.
% data = struct('x', x_out, 'theta', theta2, 'vel', v_out, ...
%                     'angvel', a_out, 'times', times, 'dt', dt, 'input', eng_RPM, 'F_des', F_des, 'thrust', thrust);
data = struct('x', x_out, 'theta', theta_out, 'vel', v_out,...
    'times', times, 'dt', dt, 'input', eng_RPM, 'F_des', F_des);% kw

%Create a full screen figure and visualize the simulation
visualize(data);