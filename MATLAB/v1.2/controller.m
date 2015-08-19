% Caculates the input and returns a vector with the four angular velocities

function angVel = controller()
    angVel = zeros(4, 1);
%      angVel(1) = 1285;
%      angVel(2) = 1285;
%      angVel(3) = 1290;
%      angVel(4) = 1290;
    angVel(:) = 50;
    angVel = angVel .^ 2;
end