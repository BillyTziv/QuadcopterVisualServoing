% Quadcopter Simulation Project.
%
% Developed by Tzivaras Vasilis
% contact me at vtzivaras@gmail.com

% Clearing previous simulation variables
clear all;
clc;

% Enviromental and quad's constants 
g = 9.81;                       % gravity acceleration
m = 0.5;                        % mass in kg
L = 0.25;                       % length of the rods
ct = 3e-6;                      % thrust coefficient
b = 1e-7;                       % torque due to drag coefficient
kd = 0.25;                      % drag coefficient
I = diag([5e-3, 5e-3, 10e-3]);  % Inertia Matrix

% Simulation start and end time 
startTime = 0;                  % Start time of the simulation
endTime = 100;                  % End time of the simulationh
dt = 0.005;                     % Steps

times = startTime:dt:endTime;   % Vector with all the times
N = numel(times);               % #of times that simulation will run

% Output values, recorded as the simulation runs
x_out = zeros(3,N);             % mass position
v_out = zeros(3,N);             % mass velocity
a_out = zeros(3,N);             % mass acceleration

eng_omega = zeros(4, N);        % KV of the four motors(INPUT)

omega = zeros(3, N);            % Angular velocities on yaw, pitch and roll
omegadot = zeros(3, N);         % Angular acceleration on yaw, pitch and roll

theta = zeros(3, N);            % Euler angles of yaw, pich, roll
thetadot = zeros(3, N);         % Derivative of yaw, pitch, roll NOT omegadot

thrust = zeros(3, N);           % vector with values only on z axis
torque = zeros (3, N);          % Torque of yaw, pitch, roll

% Run the simulation loop and everytime calculate x,y,z and angles and
% store them at the right position of the vector according to the index
index = 1;
for t = times
    % Get the input from the controller (omega - angular velocity of the 
    % motors)
    eng_omega(:, index) = controller(index); 

    % Calculate the accelerations of the mass - psi, phi, theta
    R = rotation(theta(1), theta(2), theta(3));
    thrust(:, index) = [0; 0; ct * sum(eng_omega(index))];
    weight = [0;0;-m*g;];
    a_out(:, index) = (weight + R*thrust(:, index))/m;
    
    % calculate the velocities of the mass
    v_out(:, index) = v_out(:, index) + dt * a_out(:, index);
    
    % calculate the positions of the mass
    x_out(:, index) = x_out(:, index) + dt * v_out(:, index);
    
    % Calculate omega dot
    torque(1, index) = 0.20*ct*(eng_omega(2)*eng_omega(2) - eng_omega(4)*eng_omega(4));
    torque(2, index) = 0.20*ct*(eng_omega(3)*eng_omega(3) - eng_omega(1)*eng_omega(1));
    torque(3, index) = -ct*(eng_omega(1)*eng_omega(1)) +ct*(eng_omega(2)*eng_omega(2))-ct*(eng_omega(3)*eng_omega(3))+ct*(eng_omega(4)*eng_omega(4));
    
    reverseI = diag([1/I(1,1), 1/I(2, 2), 1/I(3, 3)]);
    omegadot(:, index) = I\(cross(-omega(:, index),I*omega(:, index))+torque(:, index));
    
    % Calculate omega
    omega(:, index) = omega(:, index) + dt * omegadot(:, index);
    
    % Calculate thetadot (yaw, pitch, roll derivatives) 
    thetadot(:, index) = omegatothetadot(omega(:, index), ...
                            theta(:, index));
    
    % Calculate theta (yaw, pitch, roll angles) 
    theta(:, index) = theta(:, index) + dt * thetadot(:, index);
    
    % Increase the index for the next loop
    index = index + 1;          
end

% Put all simulation variables into an output struct.
data = struct('x', x_out, 'v', v_out, 'a', a_out, 'theta', theta, 'angVel', omega, 'input', eng_omega, 'times', times);
                %'angvel', thetadot, 't', t, 'dt', dt, 'input', engAngVel);

%visualize_test(data);

% Create a new figure and visualize the simulation
figure;
for index = 1:40:length(times)
    visualize(x_out(1, index), x_out(2, index), x_out(3, index));
    drawnow;
    if index < length(times)
    	clf;
    end
end
