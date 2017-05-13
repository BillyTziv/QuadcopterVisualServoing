%=================================================
% This project simulates a quadcopter in outdoor enviroment using IBVS
% Computer Science and Engineering,
% Department, University of Ioannina
% Developed by Tzivaras Vasilis
% Contact v t z i v a r a s @ gmail.com
%
% Computer Science and Engineering Department
% University of Ioannina
% Year 2017
% The following code is developed by Tzivaras Vasilis
% and is under GNU General Public License v2.0
%
% Feel free to contact me at vtzivaras@gmail.com for any bugs
% =======================================

% Clearing previous simulation variables and close figures
clear all; close all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Enviromental and quadcopter constants  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = 9.81;                       % gravity acceleration m/s^2
m = 1;                          % mass in gram
d = 0.25;                       % length of the rods

ct = 3e-6;                      % thrust coefficient N
cq = 1e-7;                     % torque due to drag coefficient N

k=1;
ktx = 1;                        % X axis torque factor
kty = 1;                         % Y axis torque factor
ktz = 1.8;                      % Z axis toruqe factor
kd = 1;
kthetax=0.02;
kthetay=0.02;
ktzd = 1.8;
kvx = 2;                        % Velocity constant in X axis
kvy = 2;                        % Velocity constant in Y axis
kvz = 2;                        % Velocity constant in X axis

% Coordinates of the principal point
c_u = 512;
c_v = 512;

% Focal length
f = 0.008;

% Gain 
lamda = 1;

% Ratio of the pixel dimensions
a = 102400;

% Estimation of Z
x1_Z = 1;
x2_Z = 1;
x3_Z = 1;
x4_Z = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation Initializations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I = diag([5e-3, 5e-3, 10e-3]);  % Inertia Matrix

% Simulation start and end time 
startTime = 0;                  % Start time of the simulation (s)
endTime = 25;                   % End time of the simulationh (s)
dt = 0.001;

times = startTime:dt:endTime;   % Vector with all the times
N = numel(times);               % #of times that simulation will run

%startPoint = [0, 0, 0];         % Starting from x, y, z (0, 0, 0)
%endPoint = [-5, 4, 10];          % Desired final position  x, y, z
theta_z_des = 0; %45*(pi/180);% kw

% Output values, recorded as the simulation runs
x_out = zeros(3,N);             % mass position [X-Y-Z]
v_out = zeros(3,N);             % mass velocity [X-Y-Z]
a_out = zeros(3,N);             % mass acceleration [X-Y-Z]
eng_RPM_out = zeros(4, N);
F_des_out = zeros(1, N);
omega_out = zeros(3, N);
omegadot_out = zeros(3, N);
theta_out = zeros(3, N);
thetadot_out = zeros(3, N);
thrust_out = zeros(3, N);
torque_out = zeros(3, N);
sdes_out = zeros(8, N);
vc_des_out = zeros(6, N);           % Camera velocity
omega_CAM = zeros(3, N);
vc_des_old_out = zeros(4, N);


% Initialize vectors with zero' s and keep
% all the data needed for the visualization 
e = zeros(8, N);
v_c_des_out = zeros(6, N);
v_c_des = zeros(6, 1);
s = zeros(8, 1);
s_out = zeros(8, N);
error_out = zeros(1, N);

% Vectors initialization
v = [0 0 0]';
x = [0 0 0]';
omega = [0 0 0]';
omegadot = [0 0 0]';
theta = [0 0 0]';
thetadot = [0 0 0]';
thrust_des = 0;
theta_x_des = 0;
thete_y_des =0;
torque = [0 0 0]';
eng_RPM = [0 0 0 0]';

p_cm1 = zeros(2, N);
p_cm2 = zeros(2, N);
p_cm3 = zeros(2, N);
p_cm4 = zeros(2, N);

% Desired position (px)
p_d_px(:, 1) = [412 412]';
p_d_px(:, 2) = [412 612]';
p_d_px(:, 3) = [612 612]';
p_d_px(:, 4) = [612 412]';

% Desired position (cm)
x_d_cm(1) = (p_d_px(1, 1) - c_u) / (f*a);
x_d_cm(2) = (p_d_px(1, 2) - c_u) / (f*a);
x_d_cm(3) = (p_d_px(1, 3) - c_u) / (f*a);
x_d_cm(4) = (p_d_px(1, 4) - c_u) / (f*a);

y_d_cm(1) = (p_d_px(2, 1) - c_v) / (f*a);
y_d_cm(2) = (p_d_px(2, 2) - c_v) / (f*a);
y_d_cm(3) = (p_d_px(2, 3) - c_v) / (f*a);
y_d_cm(4) = (p_d_px(2, 4) - c_v) / (f*a);

% Position vector in pixels
% p_px(:, 1) = [280 469]';
% p_px(:, 2) = [318 524]';
% p_px(:, 3) = [373 486]';
% p_px(:, 4) = [335 431]';

% p_px(:, 1) = [212 112]';
% p_px(:, 2) = [212 312]';
% p_px(:, 3) = [412 312]';
% p_px(:, 4) = [412 112]';

p_px(:, 1) = [8*612 8*412]';
p_px(:, 2) = [8*612 8*612]';
p_px(:, 3) = [8*812 8*612]';
p_px(:, 4) = [8*812 8*412]';

% Position vector in cm
x_cm(1) = (p_px(1, 1) - c_u) / (f*a);
x_cm(2) = (p_px(1, 2) - c_u) / (f*a);
x_cm(3) = (p_px(1, 3) - c_u) / (f*a);
x_cm(4) = (p_px(1, 4) - c_u) / (f*a);

y_cm(1) = (p_px(2, 1) - c_v) / (f*a);
y_cm(2) = (p_px(2, 2) - c_v) / (f*a);
y_cm(3) = (p_px(2, 3) - c_v) / (f*a);
y_cm(4) = (p_px(2, 4) - c_v) / (f*a);   

s_des = [x_d_cm(1), y_d_cm(1), x_d_cm(2), y_d_cm(2), x_d_cm(3), y_d_cm(3),...
        x_d_cm(4), y_d_cm(4)]';
s = [x_cm(1), y_cm(1), x_cm(2), y_cm(2), x_cm(3), y_cm(3), x_cm(4), y_cm(4)]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for index = 1:1:length(times)
    % Moore - Penrose Pseudoinverse interaction matrix. We need one matrix
    % fore each interesting point. So for 4 points we have Lx with 4
    % matrix vectors.
    Lx1 = [-1/x1_Z, 0, x_cm(1)/x1_Z, x_cm(1)*y_cm(1), -(1+x_cm(1)^2),...
        y_cm(1); 0, -1/x1_Z, y_cm(1)/x1_Z, 1+y_cm(1)^2, -x_cm(1)*y_cm(1), -x_cm(1);];
    Lx2 = [-1/x2_Z, 0, x_cm(2)/x2_Z, x_cm(2)*y_cm(2), -(1+x_cm(2)^2),...
        y_cm(2); 0, -1/x2_Z, y_cm(2)/x2_Z, 1+y_cm(2)^2, -x_cm(2)*y_cm(2), -x_cm(2);];
    Lx3 = [-1/x3_Z, 0, x_cm(3)/x3_Z, x_cm(3)*y_cm(3), -(1+x_cm(3)^2),...
        y_cm(3); 0, -1/x3_Z, y_cm(3)/x3_Z, 1+y_cm(3)^2, -x_cm(3)*y_cm(3), -x_cm(3);];
    Lx4 = [-1/x4_Z, 0, x_cm(4)/x4_Z, x_cm(4)*y_cm(4), -(1+x_cm(4)^2),...
        y_cm(4); 0, -1/x4_Z, y_cm(4)/x4_Z, 1+y_cm(4)^2, -x_cm(4)*y_cm(4), -x_cm(4);];
    
    Lx = [Lx1; Lx2; Lx3; Lx4;];

    % Calculate the difference between s and s_des
    error = s - s_des;
    
    % Velocity and angular velocity of the camera
    v_c_des = -lamda * pinv(Lx) * error;

	% Quadcopter has a total thrust coming from the four motors. Here we
    % calculate the desired total thrust that we need according to an error
    % calculated fromt he difference between the current and the desired
    % position of the vehicle. (PD controller)
     thrust_des = (m*g) + k*(v_c_des(3) - v(3));
	 
	
    % Change the coordinate system of the desired velocity, from the rigit
    % body frame to velocity coordinate system.
    R_VCS = [cos(theta(1)), -sin(theta(1)), 0;
            sin(theta(1)), cos(theta(1)), 0;
            0, 0, 1;];
       
    v_des = diag([kvx kvy kvz])*([v_c_des(1);v_c_des(2);v_c_des(3)] - v) - diag([15 15 15])*v;
    v_des_VCS = R_VCS'*v_des;
    v_VCS = R_VCS'*v;
    
    theta_x_des = -(m/thrust_des)*kthetax*(v_des_VCS(2) - v_VCS(2));
    torque(1) = ktx*5*(theta_x_des-theta(2))-ktx*thetadot(2);
    
    theta_y_des = (m/thrust_des)*kthetay*(v_des_VCS(1) - v_VCS(1));
    torque(2) = ktx*5*(theta_y_des-theta(3))-ktx*thetadot(3);

    torque(3) = ktz*(theta_z_des-theta(1))+ktzd*(0-thetadot(1));

    vector = [thrust_des, torque(1), torque(2), torque(3)]';
    gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
    eng_RPM = sqrt(abs(gamma\vector));

    R = rotation(theta);
    thrust(:, index) = [0; 0; ct * sum(eng_RPM.^2)];
   
    weight = [0;0;-m*g;];
    a = (weight + R*thrust(:, index))/m;
    v = v + dt * a;
    x = x + dt * v;
    
    omegadot = I\(cross(-omega,(I*omega))+torque(:));
    omega = omega + dt * omegadot;
    
    thetadot = omega2thetadot(omega,theta);
    theta = theta + dt * thetadot;
    
    % Store the values
    theta_out(:, index) = theta;
    x_out(:, index) = x;
    v_out(:, index) = v;
    a_out(:, index) = a;
    eng_RPM_out(:, index) = eng_RPM;
    F_des_out(:, index) = thrust_des;
    torque_out(:, index) = torque;
	
	
	% Calculate the position of the camera
    s_dot = Lx * [v;omega];
    s = s + dt * s_dot;
    
    % Save all the current pixel positions so we can visualize later on
    s_out(:, index) = s;
	
end


figure('units','normalized','outerposition',[0 0 1 1], 'KeyPressFcn', @visualize);
data = struct('x', x_out, 'v', v_out, 'a', a_out, 'torque', torque_out, 'theta', theta_out,...
    'times', times, 'dt', dt, 'eng_RPM', eng_RPM_out, 'F_des', F_des_out,'s_d_out', s_out, 's_des', s_des);

visualize_FULL(data);