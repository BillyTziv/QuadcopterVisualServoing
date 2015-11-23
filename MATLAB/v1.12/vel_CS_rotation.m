% This function calculates the desired and currect craft velocity to a new
% coordinate system.
%
% Rotation matrix follows the order:
% Z     X       Y
% Yaw   Roll    Pitch
% psi   phi     theta

function [gnd_CS_des_vel, gnd_CS_vel] = vel_CS_rotation(des_vel, v, theta)
    R = [cos(theta(1)), -sin(theta(1)), 0;sin(theta(1)), cos(theta(1)), 0;0, 0, 1;];
    gnd_CS_des_vel = [0 0]';
    % Desired velocities according to the new coordinate system
    gnd_CS_des_vel = R*des_vel;

    % Craft velocities according to the new coordinate system
    gnd_CS_vel = R*v;
end