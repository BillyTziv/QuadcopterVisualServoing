% Quadcopter Simulator

% Physical constants.
mass = 1.4;
gravity = 9.81;
length = 0.25;

k = 3e-6;
b = 1e-7;
I = diag([5e-3, 5e-3, 10e-3]);
kd = 0.25;

weight = [0; 0; -gravity*mass];
omega = [20;20;20;20];


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
dt = 0.5;

times = start_time:dt:end_time;
N = numel(times);

% 3xN vectors
a = zeros(3, N);
v = zeros(3, N);
x = zeros(3, N);

inp = zeros(4, N);

counter = 1;
for t = times
    inp(:, counter) = in();
    % Calculate the acceleration vector
    a(:, counter) = acceleration(phi, psi, theta, k, mass, weight, inp(counter));
    
    % Calculate the velocity vector
    v(:, counter) =  v(:, counter) + dt * v(:, counter);
    
    
    % Calculate the position vector
    x(:, counter) =  x(:, counter) + dt * x(:, counter);
    
    
    counter = counter+1;
end

subplot(3,1,1);plot(times, a(3, :), 'r', 'LineWidth', 2);grid;ylabel('Acc. in [m/s^2]');
subplot(3,1,2);plot(times, v(3, :), 'b', 'LineWidth', 2);grid;ylabel('Vel. in [m/s]');
subplot(3,1,3);plot(times, x(3, :), 'g', 'LineWidth', 2);grid;xlabel('Time in [s]');ylabel('Pos. in [m]');

%figure; plot(times, a(3, :), 'r', 'LineWidth', 2);
%figure; plot(times, v(3, :), 'b', 'LineWidth', 2)
%figure; plot(times, x(3, :), 'g', 'LineWidth', 2)

















