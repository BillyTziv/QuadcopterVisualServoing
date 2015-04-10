% This code is a simulator for UAV Quadcopters
% Input omega1 to omega 4
% Output position, velocity and acceleration of the quadcopter

% Quadcopter and enviroment defines
% Weight: 1.4 Kg
% Gravity: 9.81
% Omega1-4: 20

mass = 1.4;
gravity = 9.81;
weight = [0;0;-mass*gravity];

omega = [20;20;20;20];
thrust = [0;0;0];

ct = 7.3;
cq = 9.1;
dct = 5;
gamma = [ct ct ct ct;0 dct 0 -dct;-dct 0 dct 0;-cq cq -cq cq];

phi = 0;
psi = 0;
theta = 0;

start_time = 0;
end_time = 10;
dt = 0.005;

times = start_time:dt:end_time;
N = numel(times);

% 3xN vectors
acceleration = zeros(3, N);
velocity = zeros(3, N);
position = zeros(3, N);

counter = 1;
for t = times
    R = [cos(phi)*cos(psi)-cos(theta)*sin(phi)*sin(psi) ...
        -cos(psi)*sin(phi)-cos(phi)*cos(theta)*sin(psi) ...
        sin(theta)*sin(psi); cos(theta)*cos(psi)*sin(phi)+cos(phi)*sin(psi) ...
        cos(phi)*cos(theta)*cos(psi)-sin(phi)*sin(psi) -cos(psi)*sin(theta); ...
        sin(phi)*sin(theta) cos(phi)*sin(theta) cos(theta)];
    
    thrust = gamma*omega;
    
    % Calculate the acceleration vector
    acceleration(:, counter) = ( weight + (R*thrust(2:4)) )/mass;
    
    % Calculate the velocity vector
    velocity(:, counter) =  integral(acceleration, 0, Inf)
    
    % Calculate the position vector
    position(:, counter) =  integral(velocity, 0, Inf)
    
    counter = counter+1;
end

figure; plot(acceleration)
figure; plot(velocity)
figure; plot(position)