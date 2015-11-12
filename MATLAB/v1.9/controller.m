% Caculates the input and returns a vector with the four angular velocities

function angVel = controller()
    angVel = zeros(4, 1);
    m=26;
    angVel(1) = 904.1571-250;
    angVel(2) = 904.1571+250;
    angVel(3) = 904.1571-250;
    angVel(4) = 904.1571+250;
end