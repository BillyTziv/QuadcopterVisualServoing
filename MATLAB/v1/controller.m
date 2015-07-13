% Caculates the input and returns a vector with the four angular velocities

function angVel = controller(index)
    angVel = zeros(4, 1);
    angVel(1) = 1320;
    angVel(2) = 1320;
    angVel(3) = 1340;
    angVel(4) = 1340;
    
    angVel = angVel .^ 2;
end