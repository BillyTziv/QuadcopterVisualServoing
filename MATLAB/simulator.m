% Quadcopter Simulator

% Physical constants.
mass = 1.4;
gravity = 9.81;
length = 0.25;
k = 3e-6;
b = 1e-7;
I = diag([5e-3, 5e-3, 10e-3]);
kd = 0.25;

weight = [0; 0; -gravity];
omega = [20;20;20;20];
thrust = [0;0;0];

ct = 7.3;
cq = 9.1;
dct = 5;
gamma = [ct ct ct ct;0 dct 0 -dct;-dct 0 dct 0;-cq cq -cq cq];

% phi psi theta angle
phi = 0;
psi = 0;
theta = 0;

start_time = 0;
end_time = 10;
dt = 0.005;

times = start_time:dt:end_time;
N = numel(times);

% 3xN vectors
a = zeros(3, N);
velocity = zeros(3, N);
position = zeros(3, N);

counter = 1;
for t = times
    i(:, counter) = in();
    % Calculate the acceleration vector
    a(:, counter) = acceleration(phi, psi, theta, k, mass, weight, i(counter));
    
    % Calculate the velocity vector
    %velocity(:, counter) =  integral(a, 0, Inf)
    
    % Calculate the position vector
    %position(:, counter) =  integral(velocity, 0, Inf)
    
    counter = counter+1;
end
figure; 
plot(a, 'b');

%figure; plot(velocity)
%figure; plot(position)