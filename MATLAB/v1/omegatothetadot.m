% Convert omega to roll, pitch, yaw derivatives
function thetadot = omegatothetadot(omega, angles)
%     phi = angles(1);
%     theta = angles(2);
%     psi = angles(3);
    %W = [1, 0, -sin(theta); 0, cos(phi), cos(theta)*sin(phi); 0, -sin(phi), cos(theta)*cos(phi)];
    %@ = [0, -sin(), cos()sin();0, cos(), sin()sin();1, 0, cos()];
    
    phi = angles(1);
psi = angles(2);
theta = angles(3);

Rx = [1, 0, 0;0, cos(phi), -sin(phi);0, sin(phi), cos(phi)];
Ry = [cos(psi), 0, sin(psi);0, 1, 0;-sin(psi), 0, cos(psi)];
Rz = [cos(theta), -sin(theta), 0;sin(theta), cos(theta), 0;0, 0, 1];

R = Rz*Rx*Ry;


thetadot = inv(R) * omega;
end