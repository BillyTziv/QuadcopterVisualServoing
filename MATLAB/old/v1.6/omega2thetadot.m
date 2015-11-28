% Convert omega to roll, pitch, yaw derivatives
function thetadot = omega2thetadot(omega, angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    W = [
        0, cos(psi), -cos(phi)*sin(psi)
        0, sin(psi), cos(phi)*cos(psi)
        1, 0, sin(phi) 
    ];
    thetadot = W\omega;
end
