% Quadcopter Simulator

% Clearing previous simulation variables
clear all;
clc;

% Enviromental and quad's constants 
g = 9.81;                       % gravity acceleration
m = 0.5;                        % mass in kg
L = 0.25;                       % length of the rods
k = 3e-6;                       % thrust coefficient
b = 1e-7;                       % torque due to drag coefficient
I = diag([5e-3, 5e-3, 10e-3]);  % moment of inertia
kd = 0.25;                      % drag coefficient

phi = 0;
psi = 0;
theta = 0;

weight = [0; 0; -g*m];

% Simulation analysis
t_start = 0;        % Start time
t_end = 15;         % End time
dt = 0.001;         % Steps

t_sim = t_start:dt:t_end;
N = numel(t_sim);

% Output values, recorded as the simulation runs
x = zeros(3,N);     % mass position
v = zeros(3,N);     % mass velocity
a = zeros(3,N);     % mass acceleration

omega = zeros(4, N);

% SIMULATION loop
index = 1;
for t = t_sim
    omega(:, index) = in();
    
    a(:, index) = acceleration(phi, psi, theta, k, m, weight, omega(index));
    v(:, index) = v(:, index) + dt * a(:, index);
    x(:, index) = x(:, index) + dt * v(:, index);
    
    index = index + 1;
end

% 3D Graph
plot3(x(1, :), x(2, :), x(3, :), 'LineWidth', 5)
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
grid on;
title('Simulating the position of a quadcopter');

% 2D plots (position, velocity, acceleration)
%subplot(3,1,1);plot(t_sim,a_out);grid;ylabel('Acc. in [m/s^2]');
%subplot(3,1,2);plot(t_sim,v_out);grid;ylabel('Vel. in [m/s]');
%subplot(3,1,3);plot(t_sim,x_out);grid;xlabel('Time in [s]');ylabel('Pos. in [m]');