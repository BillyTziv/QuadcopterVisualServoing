% Caculates the input and returns a vector with the four angular velocities

function angVel = controller(index)
    % xwrizw to diasthma (10 sec) se 2, 8, 2
    % kai kanw epitax omalh epivrad kinhsh
    % analoga me to index, stelnw diaforetikes
    % gwniakes taxythtes sto sumulator

    angVel = zeros(4, 1);
    angVel(1) = 1200+index*2;
    angVel(2) = 1200+index*2;
    angVel(3) = 1200+index*2;
    angVel(4) = 1200+index*2;
    
    angVel = angVel .^ 2;
end