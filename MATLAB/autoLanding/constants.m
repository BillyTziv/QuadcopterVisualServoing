% Enviromental and quadcopter constants  
g = 9.81;                       % gravity acceleration m/s^2
m = 1;                          % mass in gram
d = 0.25;                       % length of the rods

ct = 3e-6;                      % thrust coefficient N
cq = 1e-7;                      % torque due to drag coefficient N
k = 1;                          % Total force factor

ktx = 0.65;                     % X axis torque factor
kty = 0.05;                     % Y axis torque factor
ktz = 1.8;                      % Z axis toruqe factor
kd = 1;
kthetax=1;                          
ktzd = 1.8;
kvx = 0.2;                        % Velocity constant in X axis
kvy = 1;                        % Velocity constant in Y axis
kvz = 1;                        % Velocity constant in X axis