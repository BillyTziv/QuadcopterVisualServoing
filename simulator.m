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

ktx = 1;                     % X axis torque factor
kty = 1;                     % Y axis torque factor
ktz = 1.8;                      % Z axis toruqe factor
kd = 1;
kthetax=0.02;
kthetay=0.02;
ktzd = 1.8;
kvx = 2;                        % Velocity constant in X axis
kvy = 2;                        % Velocity constant in Y axis
kvz = 2;                        % Velocity constant in X axis

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation Initializations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = diag([5e-3, 5e-3, 10e-3]);  % Inertia Matrix

% Simulation start and end time 
startTime = 0;                  % Start time of the simulation (s)
endTime = 50;                   % End time of the simulationh (s)
dt = 0.001;

times = startTime:dt:endTime;   % Vector with all the times
N = numel(times);               % #of times that simulation will run

startPoint = [0, 0, 0];         % Starting from x, y, z (0, 0, 0)
endPoint = [-2, 4, 10];          % Desired final position  x, y, z
theta_z_des = 45*(pi/180);% kw

% Output values, recorded as the simulation runs
x_out = zeros(3,N);             % mass position [X-Y-Z]
v_out = zeros(3,N);             % mass velocity [X-Y-Z]
a_out = zeros(3,N);             % mass acceleration [X-Y-Z]
eng_RPM_out = zeros(4, N);
F_des_out = zeros(1, N);
omega_out = zeros(3, N);
omegadot_out = zeros(3, N);
theta_out = zeros(3, N);
thetadot_out = zeros(3, N);
thrust_out = zeros(3, N);
torque_out = zeros(3, N);




% eng_RPM = zeros(4, N);          % KV of the four motors(INPUT)
% omega = zeros(3, N);            % Angular velocities on yaw, pitch and roll
% omegadot = zeros(3, N);         % Angular acceleration on yaw, pitch and roll
% theta = zeros(3, N);            % Euler angles of yaw, pich, roll
% thetadot = zeros(3, N);         % Derivative of yaw, pitch, roll NOT omegadot
% 
% thrust = zeros(3, N);           % vector with values only on z axis
% torque = zeros (3, N);          % Torque of yaw, pitch, roll
% F_des = zeros(1, N);            % Error from the desire position 
% 
% theta_des = zeros(3, N);
% theta_des = 0;
% theta_out = zeros(3, N);
% vel_x_des = zeros(1, N);
% theta_x_des = zeros(1, N);
% theta_y_des = zeros(1, N);



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
omegadot = [0 0 0]';% kw
theta = [0 0 0]';% kw
thetadot = [0 0 0]';% kw
F_des = 0;
theta_x_des = 0;
thete_y_des =0;
torque = [0 0 0]';% kw
eng_RPM = [0 0 0 0]';

for index = 1:1:length(times)
    F_des = (m*g)+k*(endPoint(3)-x(3))+kd*(0-v(3));% kw

    R_VCS = [cos(theta(1)), -sin(theta(1)), 0;sin(theta(1)), cos(theta(1)), 0;0, 0, 1;];
    v_des = diag([kvx kvy kvz])*(endPoint' - x) - diag([15 15 15])*v;
    v_des_VCS = R_VCS'*v_des;
    v_VCS = R_VCS'*v;
    
    theta_x_des = -(m/F_des)*kthetax*(v_des_VCS(2) - v_VCS(2));
    torque(1) = ktx*(theta_x_des-theta(2))-ktx*thetadot(2);
    
    theta_y_des = (m/F_des)*kthetay*(v_des_VCS(1) - v_VCS(1));
    torque(2) = ktx*(theta_y_des-theta(3))-ktx*thetadot(3);

    torque(3) = ktz*(theta_z_des-theta(1))+ktzd*(0-thetadot(1));% kw

    vector = [F_des, torque(1), torque(2), torque(3)]';
    gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
    eng_RPM = sqrt(abs(gamma\vector));% kw

    R = rotation(theta);
    thrust(:, index) = [0; 0; ct * sum(eng_RPM.^2)];
   
    weight = [0;0;-m*g;];
    a = (weight + R*thrust(:, index))/m;% kw
    v = v + dt * a;
    x = x + dt * v;
    
    omegadot = I\(cross(-omega,(I*omega))+torque(:));% kw
    omega = omega + dt * omegadot;% kw
    thetadot = omega2thetadot(omega,theta);% kw
    theta = theta + dt * thetadot;% kw
    
    % Store the values
    theta_out(:, index) = theta;% kw
    x_out(:, index) = x;% kw
    v_out(:, index) = v;% kw
    a_out(:, index) = a;% kw
    eng_RPM_out(:, index) = eng_RPM;
    F_des_out(:, index) = F_des;
    torque_out(:, index) = torque;
end

figure('units','normalized','outerposition',[0 0 1 1], 'KeyPressFcn', @visualize);
% data = struct('x', x_out, 'theta', theta_out, 'vel', v_out,...
%     'times', times, 'dt', dt, 'input', eng_RPM_out, 'F_des', F_des_out);% kw
data = struct('x', x_out, 'v', v_out, 'a', a_out, 'torque', torque_out, 'theta', theta_out,...
    'times', times, 'dt', dt, 'eng_RPM', eng_RPM_out, 'F_des', F_des_out);

visualize(data);


