%===========================================
% Autonomous Quadcopter Landing using Visual Servoing
% Developed by Tzivaras Vasilis [vtzivaras@gmail.com]
% All rights reserved 
%
% Latest Update: 2 December 2017
%===========================================

% Clearing previous simulation data
clear all; clc;

disp('===========================');
fprintf('Quadrotor Landing using Visual Servoing\n');
disp('===========================');
fprintf('\n');

% Initial robot eule r angles (pitch, yaw, roll);
pitch_rob = 0;
roll_rob = 0;
yaw_rob = 25;

% Initial robot position in 3D space
posX_rob = 15.2;
posY_rob = 5.3;
posZ_rob = 15;

% Set NOISE to 0 for NO air distarbances or 1 for some constant and white
% gausian air distabances.
AIR_DISTR = 0;

% Parameters for the IBVS controller.
vc1 = 0.16;
vc2 = 0.16;
vc3 = 0.03;
vc4 = 0.05;

% Parameter for the thrust controller.
k_thrust_p = 0.6;
k_thrust_d = 0.5;
k_thrust_i = 0.32;

k_torqx_p = 3.5;
k_torqx_d = 0.5;
k_torqx_i = 0.2;

k_torqy_p = 3.5;
k_torqy_d = 0.5;
k_torqy_i = 0.2;

% Initial robot position expressed in Inertial coodinate system
pos = [posX_rob posY_rob posZ_rob]';

% Desired robot position expressed in the Inertial coordinate system.
posX_des = 2.5;
posY_des = 2.5;

pos_des = [posX_des posY_des 3.4]';

% Platform velocity. Zero vector indicates that the platform is not moving
% at all. By combining the X, Y vectors we can have a velocity in any
% direction we desire. Specify only the X or Y value. Z is always 0.
vel_platform = [0 0 0]';
if( (vel_platform(1)~= 0) || (vel_platform(1)~= 0) || (vel_platform(1)~= 0))
    fprintf('We got a moving landing platform.\n');
else
    fprintf('Landing platform is steady.\n');
end
% Update the initial position of the platform, which is exactly the
% position of the desired target points.
pos_platform = pos_des;

% Initial robot theta angles (yaw, pitch, roll)
theta = [(yaw_rob * pi)/180 (roll_rob * pi)/180 (pitch_rob * pi)/180]';
fprintf('Initial(deg) Pitch: %d Yaw: %.2d Roll: %d\n', pitch_rob, yaw_rob, roll_rob);
theta_des = [0 0 0]';

% Target points positions expressed in Inertial coodinate system
targetPoints = [(posX_des-0.5) (posY_des+0.5) 3;(posX_des+0.5)...
	(posY_des+0.5) 3;(posX_des+0.5) (posY_des-0.5) 3;(posX_des-0.5)...
	(posY_des-0.5) 3;]';

if(AIR_DISTR==0)
    fprintf('Clear sky, without air distarbances.\n');
else
    fprintf('Air disturbances activated.\n');
end

fprintf('\nInitializing DC Motor constants...');
omega_dot_mot = 0;
KV = 980;             % RPM/V
Kt1 = 30/(pi*KV);
i0 = 0.2;             % mA
J = 0.01;
i_mot = [0 0 0 0]';
v_mot_max = 12;
fprintf(' OK\n');

% Initial motors angular velocity (HOVER STATE)
omega_mot=[910 910 910 910]';
rpm_mot = (omega_mot*60)/(2*pi);
v_mot = rpm_mot/KV;

% initial variables;
fprintf('Initializing quadcopter constants...');
g = 9.81;                      % gravity acceleration m/s^2
m = 1;                          % mass in kg
d = 0.25;                       % length of the rods

ct = 3e-6;                      % thrust coefficient N
cq = 1e-7;                      % torque due to drag coefficient N
k = 10;                         % Total force factor

ktx = 0.811;                    % X axis torque factor
kty = 0.1;                      % Y axis torque factor
ktz = 1.8;                      % Z axis toruqe factor
kd = 0.1;

kthetax = 0.2;
kthetay = 0.2;
ktzd = 1.8;

kvx = 2;                        % Velocity constant in X axis
kvy = 2;                        % Velocity constant in Y axis
kvz = 2;                        % Velocity constant in X axis
I = diag([5e-3, 5e-3, 10e-3]);  % Inertia Matrix
fprintf(' OK\n');

fprintf('Initializing camera constants...');
c_u = 512;                      % X coordinate of the principal point (pixels)
c_v = 512;                      % Y coordinate of the principal point (pixels)
f = 0.008;                      % Focal length of the camera (meters)
a = 102400;                     % Ratio of the pixel dimensions (meters)
x1_Z = 1;                       % Estimation of Z depth
x2_Z = 1;                       % Estimation of Z depth
x3_Z = 1;                       % Estimation of Z depth
x4_Z = 1;                       % Estimation of Z depth
fprintf(' OK\n');

fprintf('Initializing simulation vectors...');
t1 = 0;                         % Start time of the simulation (s)
t2 = 50;                       % End time of the simulationh (s)
dt = 0.001;                     % Time dt
times = t1:dt:t2;                 % Vector with all the times
N = numel(times);                 % #of times that simulation will run
fprintf('OK\n');

fprintf('Initializing loop vectors with zero...');

% About Camera
s_error = [0 0 0 0 0 0 0 0]';
v_c = [0 0 0 0]';
omega_c_des = [0 0 0]';

% About Robot
vel = [0 0 0]';
acc = [0 0 0]';

omega = [0 0 0]';
omegadot = [0 0 0]';

thetadot = [0 0 0]';

thrust = [0 0 0]';
torque = [0 0 0];
torq_des = [0 0 0]';
omega_i_des = [0 0 0 0]';
omega_i = [0 0 0 0]';

posZ_des = 0;
posZ = 0;

thetax_des_i = 0;
thetax_i  = 0;

thetay_des_i = 0;
thetay_i  = 0;

% About the motor
omega_mot_des = [0 0 0 0]'; % Desired motor angular velocity
fprintf(' OK\n');

fprintf('Initializing vectors for the visualization...');

% About the robot
error_out = zeros(8, N);
torque_out = zeros(3, N);
thrust_out = zeros(3, N);
thrustclear_out = zeros(3, N);
thrust_des_out = zeros(3, N);
theta_out = zeros(3, N);
pos_out = zeros(3, N);
acc_out = zeros(3, N);
vel_out = zeros (3, N);

% About the cameraomega_out = zeros(3, N);
omega_C_out = zeros(3, N);
v_c_des_B_out = zeros (3, N);
cam_yaw_out = zeros(1, N);
pos_c_out = zeros(3, N);
v_c_out = zeros(4, N);
omega_c_des_out = zeros(3, N);
s_out = zeros(8, N);
s_des_out = zeros(8, N);

% About the motors
v_mot_out = zeros(4, N);
i_mot_out = zeros(4,N);
omega_mot_out = zeros(4,N);
rpm_mot_out = zeros(4, N);
rpm_mot_des_out = zeros(4, N);
omega_mot_des_out = zeros(4, N);
data = zeros(5, N);
fprintf(' OK\n');

fprintf('Initializing the rotation matrices...');

% Camera to bodyframe rotation matrix (Rz(180))
R1 = [1 0 0; 0 -1 0; 0 0 -1;];

% Bodyframe to Inertial rotation matrix (R(theta))
R = rotation(theta);
fprintf('OK\n');

% Current target points projected in the camera frame.
fprintf('Calculate the initial s vector...')  
s = calCamProjection(pos, theta, targetPoints);
fprintf('OK\n')  

% Desired target points projected in the camera frame.
fprintf('Calculate the desired s vector...')  
s_des = calCamProjection(pos_des, theta_des, targetPoints);
fprintf('OK\n\n');

% Outputs s and s_des for debugging purposes (can ignore)
printInitialCameraPoints(s, s_des);

% Visualizes s, s_des and s_des_ps for debugging purposes (can ignore)
printInitialInertialPoints(s, s_des);

% Press a key here.You can see the message 'Paused:
%disp('Press any key to continue...')  
%pause;

% Run the simulation calculations N times with step equal to 1.
fprintf('Running the simulation process...');
for index = 1:1:N
	% Applying the integral to the velocity to calculate the position of
	% the platform.
	pos_platform = pos_platform + dt*vel_platform;
	
	% Desired robot position expressed in the Inertial coordinate system.
	posX_des = pos_platform(1);
	posY_des = pos_platform(2);
	pos_des = [posX_des posY_des 3.4]';
		
	% Update the target points position expressed in Inertial coodinate
	% system.
	targetPoints = [(posX_des-0.5) (posY_des+0.5) 3;(posX_des+0.5)...
			(posY_des+0.5) 3;(posX_des+0.5) (posY_des-0.5) 3;(posX_des-0.5)...
			(posY_des-0.5) 3;]';

	% Calculate the error between the current and the desired position of
	% the robot expressed in the inertial coordinate system. The error is
	% only calculated in Z axis and we have made the assumption that the
	% landing platform (targetPoints) has zero incl?ine.
	
	% Find the target points center point.
	targetPoints_ctrX = ( targetPoints(1, 1) + targetPoints(3, 1) )/2;
	targetPoints_ctrY = ( targetPoints(1, 2) + targetPoints(3, 2) )/2;
	targetPoints_ctrZ = targetPoints(1, 3);

	pos_error= (pos(3) - targetPoints(1, 3));
	
	% This function returns 1 if the current mode is the decending phase,
	% where the robot has control, 2 if the phase is the power cut off
	% and 3 if the current status is idle.
    MODE = checkLanding(s_error, vel, index, pos_error);
	
	% The robot is idle, without any movement or rotations
	if(MODE == 3)
		omega_mot = [0 0 0 0]';
		acc = [0 0 0]';
		vel = vel_platform;
        pos = pos + vel*dt;
	else
		% The robot is fully functional and IBVS system is activated.
		
		% Descending phase using IBVS system.
		if( MODE == 1 )
			% Calculate the two Interation matrices L_1 and L_2 based on the current position
			% of the target points projection in the camera frame.
			[L_1, L_2] = calcJacob(s);
			
			% Error between the current and the desired projected target
			% points in the camera frame
			s_error = s - s_des;
			
			% Bodyframe to Inertial rotation matrix based on the current theta
			% euler angles.
			R = rotation(theta);
			
			% Express omega from the inertial to the camera frame
			% R is a rotation matrix from bodyframe to inertial.
			% R1 is a rotation matrix from camera frame to bodyframe.
			% The total (R * R1)' is the rotation matrix from the Inertial to the
			% camer a frame.
			omega_C = (R * R1)' * omega;
			
			% Desired camera velocity [vx, vy, vz, omegaz] expressed in camera
			% frame.
			v_c =(-diag([vc1 vc2 vc3 vc4]) * pinv(L_1) * s_error) - (pinv(L_1) * L_2 * [omega_C(1); omega_C(2)]);
			
			% Express the desired [vx vy vz] vector from the camera frame to the body frame.
			v_c_des_B = R1 * v_c(1:3);
			
			% Rotation matrix on Z axis about the new velocity coordinate system
			R2 = [cos(theta(1)) -sin(theta(1)) 0;sin(theta(1)) cos(theta(1)) 0;0 0 1];
			
			% Desired camera (and quadcopter) velocity expressed in the velocity
			% coordinate system
			v_c_des_V = R2' * R * R1 * v_c(1:3);
		elseif(MODE == 2)
			% Descending with steady and fixed velocity. IBVS is
			% deactivated here.
			v_c_des_B(3) = -0.8;
			v_c_des_V(2) = 0;
			v_c_des_V(1) = 0;
		end
		
		% Express the current drone velocity from the inertial to the bodyframe
		vel_quad_B = R' * vel;
		
		% Rotation matrix on Z axis about the new velocity coordinate system
		R2 = [cos(theta(1)) -sin(theta(1)) 0;sin(theta(1)) cos(theta(1)) 0;0 0 1];
		
		% Express the current drone velocity from the inertial to the velocity
		% coordinate system
		vel_quad_V = R2' * vel;
		
		% PID velocity controller for the total quadcopter thrust.
		posZ_des = posZ_des + dt * v_c_des_B(3);
		posZ = posZ + dt * vel_quad_B(3);
		thrust_des = (m*g) + k_thrust_p*(v_c_des_B(3) - vel_quad_B(3)) - k_thrust_d*acc(3) + k_thrust_i*(posZ_des - posZ);
		
		% Calculate the desired theta (roll) angle via a P velocity controller in X axis.
		theta_x_des = -0.6*(m/thrust_des)*(v_c_des_V(2) - vel_quad_V(2));
		
		% PID controller for the torque in X axis
		thetax_des_i = thetax_des_i + dt*theta_x_des; % Integral of the desired theta angle in X axis.
		thetax_i = thetax_i + dt * theta(2); % Integral of theta angle in X axis;
		torq_des(1) = k_torqy_p * (theta_x_des - theta(2)) - k_torqy_d*thetadot(2) + k_torqy_i*(thetax_des_i - thetax_i);
		
		% Calculate the desired theta (pitch) angle via a P velocity controller in Y axis.
		theta_y_des = 0.6*(m/thrust_des)*(v_c_des_V(1) - vel_quad_V(1));
		
		% PD controller for the torque in Y axis.
		thetay_des_i = thetay_des_i + dt*theta_y_des; % Integral of the desired theta angle in Y axis.
		thetay_i = thetay_i + dt * theta(1); % Integral of theta angle in Y axis;
		torq_des(2) = k_torqx_p*(theta_y_des - theta(3)) - k_torqx_d*thetadot(3) + k_torqx_i*(thetay_des_i - thetay_i);
		
		% Convert the current angular velocities to theta dot
		thetadot2 = omega2thetadot([omega(1) -omega(2) -v_c(4)]', theta);
		
		% P controller for the torque in Z axis.
		torq_des(3) = 1.5*(thetadot2(1) - thetadot(1));
		
		vector = [thrust_des, torq_des(1), torq_des(2), torq_des(3)]';
		gamma = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0; -cq, cq, -cq, cq];
		
		% Calculate the desired motors RPM
		omega_mot_des = sqrt(abs(gamma\vector));
		
		% Calculates the angular velocity of each motor.
		% Calculate the ammount of voltage in the DC motor
		omega_i_des = omega_i_des + dt * omega_mot_des;
		omega_i = omega_i + dt * omega_mot;

		v_motch = 0.1*(omega_mot_des - omega_mot);% + 2.5*(omega_i_des - omega_i);
		v_mot = v_mot + v_motch;
		
		% Set motor voltage constrains (LOW and HIGH limit)
		if( v_mot(1)<0)
			v_mot(1)=0;
		end
		if( v_mot(2)<0)
			v_mot(2)=0;
		end
		if( v_mot(3)<0)
			v_mot(3)=0;
		end
		if( v_mot(4)<0)
			v_mot(4)=0;
        end
        
		if(v_mot(1)>v_mot_max)
			v_mot(1)=v_mot_max;
		end
		if(v_mot(2)>v_mot_max)
			v_mot(2)=v_mot_max;
		end
		if(v_mot(3)>v_mot_max)
			v_mot(3)=v_mot_max;
		end
		if(v_mot(4)>v_mot_max)
			v_mot(4)=v_mot_max;
		end
		
		% Calculate the ammount of current passing through the coils
		i_mot = (v_mot - Kt1*omega_mot)/0.35;
		
		% Calculate the motor angular acceleration of each motor
		omega_dot_mot = Kt1*(i_mot - i0)/J;
		omega_mot_prev = omega_mot;
		
		omega_mot = omega_mot + dt*omega_dot_mot;
		
		% Calculate the thrust force of the robot in Z axis, WN is the
		% enviromental noise from wind.
		if(AIR_DISTR == 1)
            % Enviromental noise at X,Y,Z axis (default is 0)
            WN = (0.1-0).*rand(3,1) - 0.05;
            airNoise = [0.5 0.5 0.5]';
			thrust = [0; 0; ct * sum(omega_mot.^2)] + airNoise + WN;
			thrustClear = airNoise + WN;
		else
			thrust = [0; 0; ct * sum(omega_mot.^2)];
		end
		
		% Calculate the real robot acceleration (INERTIAL)
		% R is the rotation matrix from the bodyframe to the inertial.
		acc = ([0; 0; -m*g;] + R*thrust) / m;
		
		% Calculate the robot velocity  (INERTIAL)+velocity from disturbances
		vel = vel + dt * acc; 
		
		% Calculate the robot position  (INERTIAL)
		pos = pos + dt * vel;
		
		% Calculate the updated omega dot
		omegadot= I\(cross(-omega, (I*omega)) + torq_des(:));
		
		% Calculate quadcopter omega from omega dot
		omega = omega + dt * omegadot;
		
		% Convert the current angular velocities to theta dot
		thetadot = omega2thetadot(omega, theta);
		
		% Find the new theta euler angles
		theta = theta + dt * thetadot;
		
		% Calculate the new s points
		s = calCamProjection(pos, theta, targetPoints);
	end
	
	% Save all the current pixel positions so we can visualize later on
	s_out(:, index) = s;
	s_des_out(:, index) = s_des;
	v_c_out(:, index) = v_c;
	error_out(:, index) = s_error;
	torque_out(:, index) = torq_des;
	thrust_out(:, index) = thrust;
	%thrustclear_out(:, index) = thrustClear;
	thrust_des_out(:, index) = thrust_des;
	theta_out(:, index) = theta;
	pos_out(:, index) = pos;
	omega_C_out(:, index) = omega_C;
	vel_out(:, index) = vel;
	acc_out(:, index) = acc;
    v_c_des_B_out(:, index) = v_c_des_B;
	v_mot_out(:, index)=v_mot;
	i_mot_out(:, index) = i_mot;
	omega_mot_des_out(:, index) = omega_mot_des;
	omega_mot_out(:, index) = omega_mot;
	mode_out(index) = MODE;
	pos_error_out(:, index) = pos_error;
	mymode_out(:, index) = MODE;
    targetPoints1_out(:, index) = targetPoints(:, 1);
    targetPoints2_out(:, index) = targetPoints(:, 2);
    targetPoints3_out(:, index) = targetPoints(:, 3);
    targetPoints4_out(:, index) = targetPoints(:, 4);
end
fprintf('OK\n');

fprintf('Running the visualization process...');

% % Open a new figure in fullscreen mode
figure('name', 'IBVS Simulation', 'units','normalized','outerposition',[0 0 1 1], 'KeyPressFcn', @visualize);

% Clear a struct will all the necessary data
data = struct('x', pos_out, 'v', vel_out, 'a', acc_out, 'torque', torque_out, 'theta', theta_out,...
	'times', times, 'dt', dt, 'F_des', thrust_out,...
	's', s_out, 's_des', s_des_out, 'mymode', mymode_out, 'targetPoints1', targetPoints1_out,...
    'targetPoints2', targetPoints2_out, 'targetPoints3', targetPoints3_out, 'targetPoints4', targetPoints4_out);

% Visualize the data
visualize_FULL(data);

fprintf('OK\n');

% End of program. Bye bye.
fprintf('\nEnd of program. Execution was successfull!\n');