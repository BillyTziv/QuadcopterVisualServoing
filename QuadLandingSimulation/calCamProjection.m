% This functions returns the target points as they are projected in the
% camera frame in x and y coordinates, expresed in cm.

function s = calCamProjection(pos, theta, targetPoints)
% Rotation matrix from the Bodyframe to the Inertial
R = rotation(theta);

% Rotation matrix from the Camera to the Bodyframe.
R1 = [1 0 0; 0 -1 0; 0 0 -1;];

% The transpose R is from the Inertial to the Camera
cpos = [(R * R1)' * (targetPoints(:, 1) - pos);...
						(R * R1)' * (targetPoints(:, 2) - pos);...
						(R * R1)' * (targetPoints(:, 3) - pos);...
						(R * R1)' * (targetPoints(:, 4) - pos);];

x_cm1 = cpos(1, 1)/cpos(3, 1);
y_cm1 = cpos(2, 1)/cpos(3, 1);
x_cm2 = cpos(4, 1)/cpos(6, 1);
y_cm2 = cpos(5, 1)/cpos(6, 1);
x_cm3 = cpos(7, 1)/cpos(9, 1);
y_cm3 = cpos(8, 1)/cpos(9, 1);
x_cm4 = cpos(10, 1)/cpos(12, 1);
y_cm4 = cpos(11, 1)/cpos(12, 1);

% Final s vector with all the target points
s = [x_cm1;y_cm1;x_cm2;y_cm2;x_cm3;y_cm3;x_cm4;y_cm4;];
end