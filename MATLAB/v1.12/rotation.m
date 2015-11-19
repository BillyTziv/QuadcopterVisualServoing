% Rotation matrix follows the order:
% Z     X       Y
% Yaw   Roll    Pitch
% psi   phi     theta

function R = rotation(angles)
    psi = angles(1);
    phi= angles(2);
    theta= angles(3);

    R(:, 1) = [
        cos(psi) * cos(theta) - sin(theta) * sin(phi) * sin(psi)
        cos(theta) * sin(psi) + cos(psi) * sin(phi)* sin(theta)
        -cos(phi) * sin(theta)
    ];
    R(:, 2) = [
        -cos(phi) * sin(psi)
        cos(psi) * cos(phi)
        sin(phi)
    ];
    R(:, 3) = [
        cos(psi)*sin(theta) + sin(psi)*sin(phi)*cos(theta)
        sin(psi)*sin(theta) - cos(psi) * sin(phi)*cos(theta)
        cos(phi)*cos(theta)
    ];
end