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

kp = 1;                        % PID controller proportional gain
ki = 0;                        % PID controller integral gain
kd = 1;                        % PID controller derivative gain

kvx = 1;                      % Desired velocity constant in X axis
kvy = 1;                      % Desired velocity constant in X axis
kthetax = 1;                    % Desired angle constant in X axis
kthetay = 1;                    % Desired angle constant in X axis

kttx = 1;                    % Torque theta constant on X axis
ktty = 1;                    % Torque theta constant on Y axis
kttz = 1;                     % Torque theta constant on Z axis

ktx = 1;                      % X axis thetadot factor
kty = 1;                      % Y axis thetadot factor
ktz = 1;                      % Z axis thetadot factor

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation Initializations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = diag([5e-3, 5e-3, 10e-3]);  % Inertia Matrix

% Simulation start and end time 
startTime = 0;                  % Start time of the simulation (s)
endTime = 40;                   % End time of the simulationh (s)
dt = 0.005;                     % Steps

times = startTime:dt:endTime;   % Vector with all the times
N = numel(times);               % #of times that simulation will run

% 4 DOF, X Y Z coordinate and direction
startPoint = [0, 0, 0];         % start point
endPoint = [5, 0, 10];          % end point

theta_x_des = 0;                % x end direction
theta_y_des = 0;                % y end direction
theta_z_des = 45*(pi/180);      % z end direction

vel_y_des = 0;
vel_x_des = 0;

% Output values, recorded as the simulation runs
weight = [0;0;-m*g;];

error = [];
x_out = zeros(3,N);                 % mass position [X-Y-Z]
v_out = zeros(3,N);                 % mass velocity [X-Y-Z]
a_out = zeros(3,N);                 % mass acceleration [X-Y-Z]
thetadot_out = zeros(3, N);
theta_out = zeros(3, N);
omega_out = zeros(3, N);            % Angular velocities on yaw, pitch and roll
omegadot_out = zeros(3, N);         % Angular acceleration on yaw, pitch and roll
thrust_out = zeros(3, N);           % vector with values only on z axis
torque_out = zeros (3, N);          % Torque of yaw, pitch, roll
engine_RPM_out = zeros(4, N);
F_des_out = zeros(3, N);

T = 0;
vector = [0 0 0 0]';
engine_RPM = [0 0 0 0]';
torque = [0 0 0]';
thrust = [0 0 0]';
a = [0 0 0]';
x = [0 0 0]';
v = [0 0 0]';
omega = [0 0 0]';
omegadot = [0 0 0]';
theta = [0 0 0]';
thetadot = [0 0 0]';            



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
error2 = inf;
error_sum = 0;
%while error2>1 && index<4000
for index = 1:1:N
    % Desired force controller
    e = endPoint' - x;
    edot = 0 - v;
    error_sum = error_sum + dt*e(3);
    T = m*g + kp*e(3) + ki*error_sum + kd*edot(3);
    
    R2 = [cos(theta(1)), -sin(theta(1)), 0;sin(theta(1)), cos(theta(1)), 0;0, 0, 1;];
    
    % Torque controller
    vel_x_des = kvx*(endPoint(1) - x(1));
    vel_x_des_new = R2*vel_x_des;
    vel_y_des_new = R2*vel_y_des;
    v_new = R2*v;
    theta_x_des = (m/T)*kthetax*(vel_x_des_new(1) - v_new(1));
   
    vel_y_des = kvy*(endPoint(2) - x(2));
    theta_y_des = (m/T)*kthetay*(vel_y_des_new(2) - v(2));
    
  
    torque(1) = kttx*(theta_x_des - theta(3)) - ktx*thetadot(3);
    torque(2) = ktty*(theta_y_des - theta(2)) - kty*thetadot(2);
    torque(3) = kttz*(theta_z_des - theta(1)) - ktz*thetadot(1);

    % Calculate the thrust according to the above desired force and torque
    vector = [T, torque(1), torque(2), torque(3)]';
    gamma1 = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
    engine_RPM = gamma1\vector; % kw
    R = rotation(theta);
    thrust = [0; 0; ct * sum(abs(engine_RPM))];

    a = (weight + R*thrust)/m;
    v = v + dt * a;
    x = x + dt * v;

    omegadot = I\(cross(-omega, I*omega) + torque);
    omega = omega + dt * omegadot;
    
    psi1 = theta(1);
    phi1 = theta(2);
    W = [
        0, cos(psi1), -cos(phi1)*sin(psi1)
        0, sin(psi1), cos(phi1)*cos(psi1)
        1, 0, sin(phi1) 
    ];
 
    thetadot = W\omega;
    theta = theta + dt * thetadot;
    
    % Store the values for later debugging
    torque_out(:, index) = [torque(1), torque(2), torque(3)]';
    x_out(:, index) = x;
    v_out(:, index) = v;
    a_out(:, index) = a;
    omegadot_out(:, index) = omegadot;
    omega_out(:, index) = omega;
    thetadot_out(:, index) = thetadot;
    theta_out(:, index) = theta;
    error(index) = norm(endPoint'-x);
    error2 = error(index);
    engine_RPM_out(:, index) = engine_RPM;
    F_des_out(:, index) = T;
end

%plot(F_des_out(end,:), 'Linewidth', 3)

% Animation
figure('units','normalized','outerposition',[0 0 1 1], 'KeyPressFcn', @visualize);
data = struct('x', x_out, 'v', v_out, 'a', a_out, 'torque', torque_out, 'theta', theta_out,...
    'times', times, 'dt', dt, 'eng_RPM', engine_RPM_out, 'F_des', T);
visualize(data);