% Rotation matrix follows the order:
% Z     X       Y
% Yaw   Roll    Pitch
% psi   phi     theta

function R = rotation(angles)
    %phi = angles(1);
    %theta= angles(2);
    %psi= angles(3);
    psi = angles(1);% kw
    phi= angles(2);% kw
    theta= angles(3);% kw
    %Rx = [1, 0, 0;0, cos(psi), -sin(psi);0, sin(psi), cos(psi)];
    %Ry = [cos(phi), 0, sin(phi);0, 1, 0;-sin(phi), 0, cos(phi)];
    %Rz = [cos(theta), -sin(theta), 0;sin(theta), cos(theta), 0;0, 0, 1];
    %R = Rz*Rx*Ry;

    R = zeros(3);
    
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