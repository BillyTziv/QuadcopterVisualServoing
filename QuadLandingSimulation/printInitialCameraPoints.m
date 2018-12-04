% This function prints the camera points X and Y coordinates in the camera
% frame
function printInitialCameraPoints(s, s_des_ps)
	fprintf('Initial and desired target points...\n');

	fprintf('P1 - X: %f -> %f\n', s(1), s_des_ps(1));
	fprintf('P1 - Y: %f -> %f\n', s(2), s_des_ps(2));
	fprintf('P2 - X: %f -> %f\n', s(3), s_des_ps(3));
	fprintf('P2 - Y: %f -> %f\n', s(4), s_des_ps(4));
	fprintf('P3 - X: %f -> %f\n', s(5), s_des_ps(5));
	fprintf('P3 - Y: %f -> %f\n', s(6), s_des_ps(6));
	fprintf('P4 - X: %f -> %f\n', s(7), s_des_ps(7));
	fprintf('P4 - Y: %f -> %f\n\n', s(8), s_des_ps(8));
end