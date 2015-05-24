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
I = inertiaMatrix();
kd = 0.25;                      % drag coefficient


weight = [0; 0; -g*m];

% Simulation analysis
t_start = 0;                    % Start time of the simulation
t_end = 5;                      % End time of the simulationh
dt = 0.001;                     % Steps

t_sim = t_start:dt:t_end;
N = numel(t_sim);

% Output values, recorded as the simulation runs
x = zeros(3,1);                 % mass position
v = zeros(3,1);                 % mass velocity
a = zeros(3,1);                 % mass acceleration

x_out = zeros(3,N);             % mass position
v_out = zeros(3,N);             % mass velocity
a_out = zeros(3,N);             % mass acceleration

angVel = zeros(4, N);

omega = zeros(3, N);
omegadot = zeros(3, N);

theta = zeros(3, N);
thetadot = zeros(3, N);
angles = zeros(3, N);

% Run the simulation loop
index = 1;

for t = t_sim
    % Get the four angular velocities as input to the system
    angVel(:, index) = controller();

    % calculate the accelerations of the mass
    [a, thrust] = acceleration(angles(1), angles(2), angles(3), k, m, weight, angVel(index));
    
    % calculate the velocities of the mass
    v = v + dt * a;
    
    % calculate the positions of the mass
    x = x + dt * v;
    
    a_out(:, index) = a;
    v_out(:, index) = v;
    x_out(:, index) = x;
    
    revI = diag([1/I(1,1), 1/I(2, 2), 1/I(3, 3)]);
    
    omegadot(:, index) = angular_acceleration();
    
    omega(:, index) = omega(:, index) + dt * omegadot(:, index);
    thetadot(:, index) = omegatothetadot(omega(:, index), angles(:, index));
    theta(:, index) = theta(:, index) + dt * thetadot(:, index);
    index = index + 1;
end

figure;
for index = 1:40:length(t_sim)
    visualize(x_out(1, index), x_out(2, index), x_out(3, index));
    drawnow;
    if index < length(t_sim)
        clf;
    end
end



% % 3D graph for the position
% index = 1;
% figure;
% for index = 1:40:length(t_sim)
%     thrust
%     weight
%     visualize(x_out(1, index), x_out(2, index), x_out(3, index));
%    
%     drawnow;
%     if index < length(t_sim)
%         clf;
%     end
% end
