% mass–spring–damper
clear all;
clc;
%F = 1; % force
%m = 1; % mass
%k = 10; % spring constant
%b = 1; % damping coefficient
mass = 1.4;
gravity = 9.81;
length = 0.25;

k = 3e-6;
b = 1e-7;
I = diag([5e-3, 5e-3, 10e-3]);
kd = 0.25;

weight = [0; 0; -gravity*mass];


% phi psi theta angle
phi = 0;
psi = 0;
theta = 0;

% initial state
x = 0;
v = 0;
t_start = 0; % simulation start time
t_end = 15; % simulation end time
dt = 0.001; % time step
t_sim = t_start:dt:t_end; % simulation time steps
N = numel(t_sim); % number of points in the simulation

% Output values, recorded as the simulation runs
x_out = zeros(1,N); % mass position
v_out = zeros(1,N); % mass velocity
a_out = zeros(1,N); % mass acceleration

% SIMULATION
index = 0;
for t = t_sim
    index = index + 1;
    inp(:, index) = in();
    a = acceleration(phi, psi, theta, k, mass, weight, inp(index));
    v = v + dt * a(3); % velocity
    x = x + dt * v; % position
    
    x_out(index) = x;
    v_out(index) = v;
    a_out(index) = a(3);
end

% PLOTS
subplot(3,1,1);plot(t_sim,a_out);grid;ylabel('Acc. in [m/s^2]');
subplot(3,1,2);plot(t_sim,v_out);grid;ylabel('Vel. in [m/s]');
subplot(3,1,3);plot(t_sim,x_out);grid;xlabel('Time in [s]');ylabel('Pos. in [m]');