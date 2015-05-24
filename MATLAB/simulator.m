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
theta = 0;
angles = zeros(3, 1);

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

% SIMULATION loop
index = 1;
for t = t_sim
    % Calculate the position, valocity and acceleration
    angVel(:, index) = in();

    [a, thrust] = acceleration(angles(1), angles(2), angles(3), k, m, weight, angVel(index));
    v = v + dt * a;
    x = x + dt * v;
    
    a_out(:, index) = a;
    v_out(:, index) = v;
    x_out(:, index) = x;
    
    revI = diag([1/I(1,1), 1/I(2, 2), 1/I(3, 3)]);
    
    % Calculate the angular velocity
    omegadot = revI*(cross(-omega(:, index), I*omega(:, index)))+revI*thrust;
    thetadot = omegatothetadot(omega, angles);
    theta = theta + dt * thetadot;
    index = index + 1;
end

% index = 1;
% figure;
% for index = 1:40:length(t_sim)
%     thrust
%     % Check if thrust is more equal or less than the weight and stop it.
%     if weight(3) < thrust(3)
%         visualize(x_out(1, index), x_out(2, index), x_out(3, index));
%         drawnow;
%         if index < length(t_sim)
%             clf;
%         end
%     else
%        
%     end
% end



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
