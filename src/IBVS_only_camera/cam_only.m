% Clearing previous simulation data
clear all;
clc;

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

% Simulation start and end time 
startTime = 0;                          % Start time of the simulation (s)
endTime = 10;                           % End time of the simulationh (s)
dt = 0.01;

times = startTime:dt:endTime;           % Vector with all the times
N = numel(times);                       % #of times that simulation will run

% Initialize vectors with zero' s and keep
% all the data needed for the visualization 
e = zeros(8, N);
v_c_des_out = zeros(6, N);
v_c_des = zeros(6, 1);
s = zeros(8, 1);
s_out = zeros(8, N);
error_out = zeros(1, N);

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

% YAW TEST
% p_px(:, 4) = [412 412]';
% p_px(:, 1) = [412 612]';
% p_px(:, 2) = [612 612]';
% p_px(:, 3) = [612 412]';

% Thrust TEST
p_px(:, 1) = [212 212]';
p_px(:, 2) = [212 812]';
p_px(:, 3) = [812 812]';
p_px(:, 4) = [812 212]';

% PITCH TEST
% pos=200;
% p_px(:, 1) = [pos+412 412]';
% p_px(:, 2) = [pos+612 612]';
% p_px(:, 3) = [pos+612 612]';
% p_px(:, 4) = [pos+412 412]';

% ROLL TEST
% pos=200;
% p_px(:, 1) = [412 pos+412]';
% p_px(:, 2) = [412 pos+612]';
% p_px(:, 3) = [612 pos+612]';
% p_px(:, 4) = [612 pos+412]';

% PITCH & ROLL TEST
% posX = 200;
% posY = 200;
% p_px(:, 1) = [posX+412 posY+412]';
% p_px(:, 2) = [posX+412 posY+612]';
% p_px(:, 3) = [posX+612 posY+612]';
% p_px(:, 4) = [posX+612 posY+412]';


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
    
    % Calculate the position of the camera
    s_dot = Lx * v_c_des;
    s = s + dt * s_dot;
    
    % Save all the current pixel positions so we can visualize later on
    s_out(:, index) = s;
    v_c_des_out(:, index) = v_c_des;
    error_out(1, index) = sum(error)/8;
end

% Visualize the simulation
xmax = zeros(1, 4);
ymax = zeros(1, 4);

xmax(1) = max(s_out(1, :));
xmax(2) = max(s_out(3, :));
xmax(3) = max(s_out(5, :));
xmax(4) = max(s_out(7, :));

x_axis_max = max(xmax);

ymax(1) = max(s_out(2, :));
ymax(2) = max(s_out(4, :));
ymax(3) = max(s_out(6, :));
ymax(4) = max(s_out(8, :));

y_axis_max = max(ymax);  
figure( 'Name', 'Visual Servoing', 'MenuBar', 'None', 'Numbertitle', 'off', 'Units', 'pixels');
xlabel('x (pixels)')
ylabel('y (pixels)')

% Animation
for index = 1:1:length(times)
    % Visualize the object that changes
    plot([s_out(1, index) s_out(3, index)], [s_out(2, index) s_out(4, index)],...
           [s_out(3, index) s_out(5, index)], [s_out(4, index) s_out(6, index)],...
           [s_out(5, index) s_out(7, index)], [s_out(6, index) s_out(8, index)],...
           [s_out(7, index) s_out(1, index)], [s_out(8, index) s_out(2, index)],...
           'Color', 'r', 'LineWidth', 2);

    % Visualize the initial object
    hold on
    plot([s_des(1) s_des(2)],[s_des(3) s_des(4)],...
        [s_des(3) s_des(4)],[s_des(5) s_des(6)],...
        [s_des(5) s_des(6)],[s_des(7) s_des(8)],...
        [s_des(7) s_des(8)],[s_des(1) s_des(2)],...
        'Color', 'b', 'LineWidth', 2);
    hold off

    axis([0-0.5 x_axis_max+0.5 0-0.5 y_axis_max+0.5]);
    pause(0.01);
end

display('End of program!');
