% Caculates the input and returns a vector with the four angular velocities

function angVel = controller()
    angVel = zeros(4, 1);
    angVel(1) = 1800;
    angVel(2) = 1500;
    angVel(3) = 1900;
    angVel(4) = 1500;
    
    angVel = angVel .^ 2;
end