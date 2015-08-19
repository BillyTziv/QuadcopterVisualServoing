% Caculates the input and returns a vector with the four angular velocities

%     angVel(1) = 1200+index*2;
%     angVel(2) = 1200+index*2;
%     angVel(3) = 1200+index*2;
%     angVel(4) = 1200+index*2;
%     angVel = 19400 .^ 2;

function angVel = controller(error)
    hover_angular_velocity = 83.7086;
    angVel = zeros(4, 1);
    
    if (error > 0)
        angVel = hover_angular_velocity+5;
    else
        angVel = hover_angular_velocity-5;
    end
end