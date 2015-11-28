% Convert omega to roll, pitch, yaw derivatives
%
% Z     X       Y
% Yaw   Roll    Pitch
% psi   phi     theta

function thetadot = omega2thetadot(omega, angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    W = [
        0, cos(psi), -cos(phi)*sin(psi)
        0, sin(psi), cos(phi)*cos(psi)
        1, 0, sin(phi) 
    ];

    omega_x = omega(1);
    omega_y = omega(2);
    omega_z = omega(3);
    omega = [omega_z omega_x omega_y]';
    thetadot = W\omega;
    thetadot = [thetadot(1) thetadot(3) thetadot(2)];
end
