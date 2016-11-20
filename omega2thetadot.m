% Convert omega to roll, pitch, yaw derivatives
%
% Z     X       Y
% Yaw   Roll    Pitch
% psi   phi     theta

function thetadot = omega2thetadot(omega, angles)
    psi = angles(1);
    phi = angles(2);
   
    W = [
        0, cos(psi), -cos(phi)*sin(psi)
        0, sin(psi), cos(phi)*cos(psi)
        1, 0, sin(phi) 
    ];

%     W = [
%         0, -sin(phi), cos(phi)*sin(theta)
%         0, cos(phi), sin(phi)*sin(theta)
%         1, 0, cos(theta) 
%     ];

%     W = [
%         0, -sin(phi), cos(phi)*sin(theta)
%         0, cos(phi), sin(phi)*sin(theta)
%         1, 0, cos(theta) 
%     ];
% 
%      W = [
%         0, cos(psi), cos(phi)*sin(theta)
%         0, sin(psi), sin(phi)*sin(theta)
%         1, 0, cos(theta) 
%     ];
% 
%     omega_x = omega(1);
%     omega_y = omega(2);
%     omega_z = omega(3);
%     omega = [omega_z omega_x omega_y]';
%     thetadot = W\omega;
%     thetadot = [thetadot(1) thetadot(3) thetadot(2)];
    thetadot = W\omega;
end
