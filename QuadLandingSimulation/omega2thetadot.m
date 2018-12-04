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
	
% 	W = [
% 		cos(psi), 0, -sin(psi)*cos(phi)
% 		sin(psi), 0, cos(psi)*cos(phi)
% 		0, 1, sin(phi)
% 	];

    thetadot = inv(W)*omega;
end
