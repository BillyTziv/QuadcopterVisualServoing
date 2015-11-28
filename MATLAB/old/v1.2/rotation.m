function R = rotation(angles)
    phi = angles(1);
    psi = angles(2);
    theta = angles(3);

    R = zeros(3);
    
    R(:, 1) = [
        cos(phi) * cos(psi) - cos(theta) * sin(phi) * sin(psi)
        cos(theta) * cos(psi) * sin(phi) + cos(phi) * sin(psi)
        sin(phi) * sin(theta)
    ];
    R(:, 2) = [
        -cos(psi) * sin(phi) - cos(phi) * cos(theta) * sin(psi)
        cos(phi) * cos(theta) * cos(psi) - sin(phi) * sin(psi)
        cos(phi) * sin(theta)
    ];
    R(:, 3) = [
        sin(theta) * sin(psi)
        -cos(psi) * sin(theta)
        cos(theta)
    ];
end