% Quadcopter Simulation Project
%
% Developed by Tzivaras Vasilis
% contact: vtzivaras@gmail.com

% Clearing previous simulation variables
clear all;
clc;

% Enviromental and quad's constants 
g = 9.81;                       % gravity acceleration
m = 0.5;                        % mass in kg
L = 0.25;                       % length of the rods
k = 3e-6;                       % thrust coefficient
b = 1e-7;                       % torque due to drag coefficient
kd = 0.25;                      % drag coefficient
I = diag([5e-3, 5e-3, 10e-3]);  % Inertia Matrix

% Simulation start and end time 
startTime = 0;                  % Start time of the simulation
endTime = 50;                   % End time of the simulationh
dt = 0.05;                      % Steps

times = startTime:dt:endTime;   % Vector with all the times
N = numel(times);               % #of times that simulation will run

% Output values, recorded as the simulation runs
x_out = zeros(3,N);             % mass position
v_out = zeros(3,N);             % mass velocity
a_out = zeros(3,N);             % mass acceleration

eng_omega = zeros(4, N);        % Angular velocities of the four motors(IN)
thrust = zeros(3, N);

omega = zeros(3, N);            % Angular velocities on yaw, pitch and roll
omegadot = zeros(3, N);         % Angular acceleration on yaw, pitch and roll

theta = zeros(3, N);            % Euler angles of yaw, pich, roll
thetadot = zeros(3, N);         % Derivative of yaw, pitch, roll NOT omegadot

telemetry = zeros(4, N);        % THROTTLE AILERON ELEVATOR RUDDER SWITCH

index = 1;
position = index;
firstTime = 0;                  % 0 for first time in loop, 1 for evey other

delete(instrfind);
arduino=serial('COM3','BaudRate',9600);
fopen(arduino);

figure;

% Run the simulation loop and everytime calculate x,y,z and angles and
% store them at the right position of the vector according to the index
for t = times
    if firstTime == 0
        position = index;
        firstTime = 1;
    else
        position = index-1;
    end
    
    % Get the input from the controller
    telemetry(:, index) = fscanf(arduino,'%d%d%d%d')';
 
    %Calculate the accelerations of the mass
    R = rotation(theta);
    thrust(:, index) = [0; 0 ; k * telemetry(1, index) * telemetry(1, index)];
    weight = [0;0;m*g;];
    
    
    a_out(:, index) = (R*thrust(:, index)-weight)/m;
    if telemetry(2, index) < 1300
        a_out(2, index) = 5;
    elseif telemetry(2, index) > 1500
        a_out(2, index) = -5;
    end
    
    if telemetry(3, index) < 1300
        a_out(1, index) = -5;
    elseif telemetry(3, index) > 1500
        a_out(1, index) = 5;
    end
    
    % calculate the velocities of the mass
    v_out(:, index) = v_out(:, position) + dt * a_out(:, index);

    % calculate the positions of the mass
    newPosition = x_out(:, position) + dt * v_out(:, index);
    if newPosition <= 0
        x_out(:, index) = 0;
    else
        x_out(:, index) = newPosition;
    end
        
    visualize(times, x_out(:, index), telemetry); drawnow;
      
    % Increase the index for the next loop
    index = index + 1;
end

fclose(arduino);
% 
% % Put all simulation variables into an output struct.
% data = struct('x', x_out, 'v', v_out, 'a', a_out, 'theta', theta, 'angVel', omega, 'input', eng_omega, 'times', times);
%                 %'angvel', thetadot, 't', t, 'dt', dt, 'input', engAngVel);
% 
% %visualize_test(data);
% 
% %Create a new figure and visualize the simulation
% figure;
% for index = 1:40:length(times)
%     visualize(x_out(1, index), x_out(2, index), x_out(3, index));
%     drawnow;
%     if index < length(times)
%         clf;
%     end
% end
