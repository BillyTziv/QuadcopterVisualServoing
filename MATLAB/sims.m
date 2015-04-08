%
% Quadcopter Simulation for MATLAB
%

start_time = 0;
end_time = 10;
dt = 0.005;
times = start_time:dt:end_time;

% Number of points in the simulation
N = numel(times);

% Initial simulation state
x = [0; 0; 10];
xdot = zeros(3, 1);
theta = zeros(3, 1);

% Simulate some disturbance in the angular velocity
% The magnitude of the deviation is in radians / sec
deviation = 100;
thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation)

% Step though the simulatio, updating the state
for t = times
    % Take input from the controller
    i = input(t);

    omega = thetador2omega(thetadot, theta);

    %Compute linear and angular accelerations
    a = acceleration(i, theta, xdot, m, g, k, kd);
    omegadot  angular_acceleration(i, omega, I, L, b, k);

    omega omega + dt * omegadot;
    thetadot = omega2thetadot(omega, theta);
    theta = theta + dt * thetadot;
    xdot = xdot + dt * a;
    x = x + dt *xdot;
end

