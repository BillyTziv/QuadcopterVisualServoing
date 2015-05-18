% Input Data from a Controller, Joystick or anything that could be
% used for that cause

function in = in()
    in = zeros(4, 1);
    in(:) = 1300;
    
    in = in .^ 2;
end