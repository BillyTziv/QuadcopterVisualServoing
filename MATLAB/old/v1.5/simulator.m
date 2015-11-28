% This project simulates a quadcopter in outdoor enviroment
%
% Developed by Tzivaras Vasilis
% Conact at vtzivaras@gmail.com

% Clearing previous simulation variables
clear all;
clc;

% Enviromental and quad's constants  
g = 9.81;                       % gravity acceleration m/s^2
m = 1;                          % mass in gram
d = 0.25;                       % length of the rodsopen m

ct = 3e-6;                      % thrust coefficient N
cq = 1e-7;                      % torque due to drag coefficient N
k = 1;                          % Total force factor
ktx = 1.2;                      % X axis torque factor
kty = 1.2;                      % Y axis torque factor
ktz = 1.2;                      % Z axis toruqe factor

kd = 1;
I = diag([5e-3, 5e-3, 10e-3]);  % Inertia Matrix

startPoint = [0, 0, 0];         % Starting from x, y, z (0, 0, 0)
endPoint = [0, 5, 10];          % Desired final position  x, y, z

% Simulation start and end time 
startTime = 0;                  % Start time of the simulation (s)
endTime = 15;                   % End time of the simulationh (s)
dt = 0.005;                     % Steps

times = startTime:dt:endTime;   % Vector with all the times
N = numel(times);               % #of times that simulation will run

% Output values, recorded as the simulation runs
x_out = zeros(3,N);             % mass position
v_out = zeros(3,N);             % mass velocity
a_out = zeros(3,N);             % mass acceleration

eng_omega = zeros(4, N);        % KV of the four motors(INPUT)
altitude = zeros(3, N);         % In every loop this variable stores the 
omega = zeros(3, N);            % Angular velocities on yaw, pitch and roll
omegadot = zeros(3, N);         % Angular acceleration on yaw, pitch and roll

theta = zeros(3, N);            % Euler angles of yaw, pich, roll
thetadot = zeros(3, N);         % Derivative of yaw, pitch, roll NOT omegadot

thrust = zeros(3, N);           % vector with values only on z axis
torque = zeros (3, N);          % Torque of yaw, pitch, roll

F_des = zeros(1, N);            % Error from the desire position 


%Run the simulation loop and everytime calculate x,y,z and angles and
% store them at the right position of the vector according to the index
flag = 0;
for index = 1:1:length(times)
    
    % First time in the loop flag is 0 and every single time after that it
    % is 1 so the else part is executed.
    %eng_omega(:, index) = controller();
    if(flag == 0)
        F_des(index) = (m*g)+k*(endPoint(3)-altitude(3, index))+kd*(0-v_out(3, index));
        torque(1, index) = ktx*(endPoint(1)-altitude(1, index))+ktx*(0-v_out(1, index));
        torque(2, index) = kty*(endPoint(2)-altitude(2, index))+kty*(0-v_out(2, index));
        torque(3, index) = ktz*(endPoint(3)-altitude(3, index))+ktz*(0-v_out(3, index));
        
        vector= [F_des(index), torque(1, index), torque(2, index), torque(3, index)]';
        gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
        eng_omega(:, index) = sqrt(gamma\vector);
        
        % Acceleration of the craft
        R = rotation(theta);
        thrust(:, index) = [0; 0; ct * sum(eng_omega(:, index).^2)];
        weight = [0;0;-m*g;];
        
        a_out(:, index) = (weight + R*thrust(:, index))/m;

        % Valocity of the craft
        v_out(:, index) = v_out(:, index) + dt * a_out(:, index);
    
        % Position of the craft
        x_out(:, index) = x_out(:, index) + dt * v_out(:, index);

        flag = 1;
    else
        F_des(index) = (m*g)+k*(endPoint(3)-altitude(3, index-1))+kd*(0-v_out(3, index-1));
        torque(1, index) = ktx*(endPoint(1)-altitude(1, index-1))+ktx*(0-v_out(1, index-1));
        torque(2, index) = kty*(endPoint(2)-altitude(2, index-1))+kty*(0-v_out(2, index-1));
        torque(3, index) = 0;%ktz*(endPoint(3)-altitude(3, index-1))+ktz*(0-v_out(3, index-1));
        
        vector = [F_des(index), torque(1, index-1), torque(2, index-1), torque(3, index-1)]';
        gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
        eng_omega(:, index) = sqrt(gamma\vector);
        
        
        % Acceleration of the craft
        R = rotation(theta);
        thrust(:, index) = [0; 0; ct * sum(eng_omega(:, index).^2)];
        weight = [0;0;-m*g;];
        
        a_out(:, index) = (weight + R*thrust(:, index))/m;
        
        % Veloctiy of the craft
        v_out(:, index) = v_out(:, index-1) + dt * a_out(:, index);
    
        % Position of the craft
        x_out(:, index) = x_out(:, index-1) + dt * v_out(:, index);
    end
    
    % Update the altitude variable for the next loop
    altitude(:, index) = x_out(:, index);
    
    % Calculate omega dot
    %torque(1, index) = d*ct*(eng_omega(1, index)^2 - eng_omega(3, index)^2);
    %torque(2, index) = 0.20*ct*(eng_omega(2, index)^2 - eng_omega(4, index)^2);
    %torque(3, index) = -cq*(eng_omega(1, index)^2) +cq*(eng_omega(2, index)^2)-cq*(eng_omega(3, index)^2)+cq*(eng_omega(4, index)^2);
    
    omegadot(:, index) = I\(cross(-omega(:, index),I*omega(:, index))+torque(:, index));
    
    % Calculate omega
    omega(:, index) = omega(:, index) + dt * omegadot(:, index);
    
    % Calculate thetadot (yaw, pitch, roll derivatives) 
    thetadot(:, index) = omega2thetadot(omega(:, index),theta(:, index));
    
    % Calculate theta (yaw, pitch, roll angles) 
    theta(:, index) = theta(:, index) + dt * thetadot(:, index);
end
figure('units','normalized','outerposition',[0 0 1 1], 'KeyPressFcn', @visualize);

% Put all simulation variables into an output struct.
data = struct('x', x_out, 'theta', theta, 'vel', v_out, ...
                    'angvel', a_out, 'times', times, 'dt', dt, 'input', eng_omega, 'F_des', F_des, 'thrust', thrust);

% Create a full screen figure and visualize the simulation


    
visualize(data);
