% This project simulates a quadcopter in outdoor enviroment
%
% Developed by Tzivaras Vasilis
% Contact me at vtzivaras@gmail.com

% Clearing previous simulation variables
clear all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Enviromental and quadcopter constants  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = 9.81;                       % gravity acceleration (m/s^2)
m = 1;                          % mass in (gram)
d = 0.25;                       % length of the rods (cm)

ct = 3e-6;                      % thrust coefficient N
cq = 1e-7;                      % torque due to drag coefficient N
k = 0.2;                        % Total force factor

ktx = 1.65;                     % X axis torque factor
kty = 0.05;                     % Y axis torque factor
ktz = 1.8;                      % Z axis toruqe factor
kd = 0.2;
kthetax = 0.8;                          
ktzd = 1.8;
kvx = 1;                      % Velocity constant in X axis
kvy = 1;                        % Velocity constant in Y axis
kvz = 1;                        % Velocity constant in X axis

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation Initializations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = diag([5e-3, 5e-3, 10e-3]);  % Inertia Matrix

% Simulation start and end time 
startTime = 0;                  % Start time of the simulation (s)
endTime = 20;                   % End time of the simulationh (s)
dt = 0.005;                     % Steps

times = startTime:dt:endTime;   % Vector with all the times
N = numel(times);               % #of times that simulation will run

% 4 DOF, X Y Z coordinate and direction
startPoint = [0, 0, 0];         % start point
endPoint = [2, 2, 10];          % end point

theta_x_des = 0;                % x end direction
theta_y_des = 0;                % y end direction
theta_z_des = 45*(pi/180);      % z end direction

vel_y_des = zeros(1, N);
vel_x_des = zeros(1, N);

% Output values, recorded as the simulation runs
weight = [0;0;-m*g;];
engine_RPM = zeros(4, N);          % KV of the four motors(INPUT)

position = zeros(3,N);             % mass position [X-Y-Z]
velocity = zeros(3,N);             % mass velocity [X-Y-Z]
acceleration = zeros(3,N);         % mass acceleration [X-Y-Z]


omega = zeros(3, N);            % Angular velocities on yaw, pitch and roll
omegadot = zeros(3, N);         % Angular acceleration on yaw, pitch and roll
theta = zeros(3, N);
thetadot = zeros(3, N);         % Derivative of yaw, pitch, roll NOT omegadot

thrust = zeros(3, N);           % vector with values only on z axis
torque = zeros (3, N);          % Torque of yaw, pitch, roll
F_des = zeros(1, N);            % Error from the desire position 
vector = zeros(4, N);

a = [0 0 0]';
x = [0 0 0]';
v = [0 0 0]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for index = 1:1:N
    % Desired force controller
    F_des(index) = (m*g)+k*(endPoint(3)-x(3))+kd*(0-v(3));% kw

    % Torque controller
    vel_x_des(index) = kvx*(endPoint(1) - x(1));
    theta_x_des = (m/F_des(index))*kthetax*(vel_x_des(index)-v(1));
    
    vel_y_des(index) = kvx*endPoint(2) - x(2);
    theta_y_des = (m/F_des(index))*kthetax*(vel_y_des(index)-v(2));
    
    if(index == 1)
        torque(1, index) = ktx*(theta_x_des - 0)-ktx*0;
        torque(2, index) = ktx*(theta_y_des - 0)-ktx*0;
    else
        torque(1, index) = ktx*(theta_x_des - theta(3, index-1)-ktx*thetadot(3, index));
        torque(2, index) = ktx*(theta_y_des - theta(2, index-1))-ktx*thetadot(2, index);
    end
    
    torque(3, index) = ktz*(theta_z_des-theta(1))+ktzd*(0-thetadot(1));% kw

    % Calculate the thrust according to the above desired force and torque
    vector(:, index) = [F_des(index), torque(1, index), torque(2, index), torque(3, index)]';
    gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
    engine_RPM(:, index) = sqrt(abs(gamma\vector(:, index))); % kw
    R = rotation(theta);
    thrust(:, index) = [0; 0; ct * sum(engine_RPM(:, index).^2)];

    % Calculate acceleration, velocity and position of the vehicle
    a = (weight + R*thrust(:, index))/m;
    v = v + dt * a;
    x = x + dt * v;
    
    position(:, index) = x;
    velocity(:, index) = v;
    acceleration(:, index) = a;
    
    % Calculate omegadot, omega, thetadot and theta
    if(index == 1)
        omegadot(:, index) = I\(cross(-omega(:, index),I*omega(:, index)) + torque(:, index));
    else
        omegadot(:, index) = I\(cross(-omega(:, index-1),I*omega(:, index-1)) + torque(:, index));
    end
    
    omega(:, index) = omega(:, index) + dt * omegadot(:, index);% kw
    
    psi = theta(1);
    phi = theta(2);
   
    W = [
        0, cos(psi), -cos(phi)*sin(psi)
        0, sin(psi), cos(phi)*cos(psi)
        1, 0, sin(phi) 
    ];
 
    thetadot(:, index) = W\omega(:, index);
    theta(:, index) = theta(:, index) + dt * thetadot(:, index);

end

figure('units','normalized','outerposition',[0 0 1 1], 'KeyPressFcn', @visualize);

data = struct('x', position, 'v', velocity, 'a', acceleration, 'torque', torque, 'theta', theta,...
    'times', times, 'dt', dt, 'eng_RPM', engine_RPM, 'F_des', F_des);

visualize(data);