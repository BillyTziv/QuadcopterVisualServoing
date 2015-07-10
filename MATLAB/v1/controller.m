% Caculates the input and returns a vector with the four angular velocities

function angVel = controller(index)
    angVel = zeros(4, 1);
    angVel(1) = 500;
    angVel(2) = 500;
    angVel(3) = 500;
    angVel(4) = 500;
    
    angVel = angVel .^ 2;
end