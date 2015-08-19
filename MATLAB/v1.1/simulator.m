% Quadcopter Simulation Project.
%
% Developed by Tzivaras Vasilis
% contact me at vtzivaras@gmail.com

% Clearing previous simulation variables
clear all;
clc;

% Enviromental and quad's constants 
g = 9.81;                       % gravity acceleration m/s^2
m = 1;                          % mass in gram
d = 0.25;                       % length of the rodsopen m
ct = 0.0014;                    % thrust coefficient N
cq = 3.2904e-07;                % torque due to drag coefficient N
k = 1;
hover_angular_velocity = 83.7086;
I = diag([5e-3, 5e-3, 10e-3]);  % Inertia Matrix

                                % current altitude of the craft m

startPoint = [0, 0, 0];         % Starting from x, y, z (0, 0, 0)
endPoint = [0, 0, 100];         % Desired final position  x, y, z (0, 0, 100)

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
altitude = zeros(1, N);                   % In every loop this variable stores the 
omega = zeros(3, N);            % Angular velocities on yaw, pitch and roll
omegadot = zeros(3, N);         % Angular acceleration on yaw, pitch and roll

theta = zeros(3, N);            % Euler angles of yaw, pich, roll
thetadot = zeros(3, N);         % Derivative of yaw, pitch, roll NOT omegadot

thrust = zeros(3, N);           % vector with values only on z axis
torque = zeros (3, N);          % Torque of yaw, pitch, roll

F_des = zeros(1, N);            % Error from the desire position 


% Run the simulation loop and everytime calculate x,y,z and angles and
% store them at the right position of the vector according to the index
index = 1;
flag = 0;
counter=0;
for t = times
    if(flag == 0)
        F_des(index) = k*(endPoint(3)-altitude(1, index))
        vector= [F_des(index), torque(1, index), torque(2, index), torque(3, index)]'
        gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq]
        eng_omega(:, index) = (gamma\vector)./2;
        
        % Calculate the accelerations of the mass - psi, phi, theta
        R = rotation(theta(1), theta(2), theta(3));
        thrust(:, index) = [0; 0; ct * sum(eng_omega(index)*eng_omega(index))];

        weight = [0;0;-m*g;];
        a_out(:, index) = (weight + R*thrust(:, index))/m;

        % calculate the velocities of the mass
        v_out(:, index) = v_out(:, index) + dt * a_out(:, index);
    
        % calculate the positions of the mass
        x_out(:, index) = x_out(:, index) + dt * v_out(:, index);
        if(x_out(:, index) <= 0)
            x_out(:, index) = 0;
        end
        flag = 1;
    else
        F_des(index) = k*(endPoint(3)-altitude(1, index-1));
        vector= [F_des(index), torque(1, index-1), torque(2, index-1), torque(3, index-1)]';
        gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
        eng_omega(:, index) = inv(gamma)*vector;
        counter = counter+1;
        % Calculate the accelerations of the mass - psi, phi, theta
        R = rotation(theta(1), theta(2), theta(3));
        thrust(:, index) = [0; 0; ct * sum(eng_omega(index)*eng_omega(index))];

        weight = [0;0;-m*g;];
        a_out(:, index) = (weight + R*thrust(:, index))/m;
        
        % calculate the velocities of the mass
        v_out(:, index) = v_out(:, index-1) + dt * a_out(:, index);
    
        % calculate the positions of the mass
        x_out(:, index) = x_out(:, index-1) + dt * v_out(:, index);
        if(x_out(:, index) <= 0)
            x_out(:, index) = 0;
        end
    end
    
    % Update the altitude variable for the next loop
    altitude(1, index) = x_out(3, index);
    
    % Calculate omega dot
    torque(1, index) = 0.20*ct*(eng_omega(2)*eng_omega(2) - eng_omega(4)*eng_omega(4));
    torque(2, index) = 0.20*ct*(eng_omega(3)*eng_omega(3) - eng_omega(1)*eng_omega(1));
    torque(3, index) = -cq*(eng_omega(1)*eng_omega(1)) +cq*(eng_omega(2)*eng_omega(2))-cq*(eng_omega(3)*eng_omega(3))+cq*(eng_omega(4)*eng_omega(4));
    
    omegadot(:, index) = I\(cross(-omega(:, index),I*omega(:, index))+torque(:, index));
    
    %thetadot2omega???
    
    % Calculate omega
    omega(:, index) = omega(:, index) + dt * omegadot(:, index);
    
    % Calculate thetadot (yaw, pitch, roll derivatives) 
    thetadot(:, index) = omega2thetadot(omega(:, index),theta(:, index));
    
    % Calculate theta (yaw, pitch, roll angles) 
    theta(:, index) = theta(:, index) + dt * thetadot(:, index);
    
    % Increase the index for the next loop
    index = index + 1;
    
end

% Put all simulation variables into an output struct.
%data = struct('x', x_out, 'v', v_out, 'a', a_out, 'theta', theta, 'angVel', omega, 'input', eng_omega, 'times', times);
%'angvel', thetadot, 't', t, 'dt', dt, 'input', engAngVel);

%visualize_test(data);

% Create a new figure and visualize the simulation
figure;
for index = 1:20:length(times)
    %visualize(times, x_out, v_out, a_out);
    old_vis(x_out(1, index), x_out(2, index), x_out(3, index), x_out, v_out, a_out, times);
    drawnow;
    if index < length(times)
    	clf;
    end
end