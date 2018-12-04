function status=checkLanding(error, vel, index, le)
counter=0;    
    for i=1:1:8
        if error(i) < 0.001
            counter = counter+1;
        end
    end
    for i=1:1:3
        if vel(i) < 0.001
            counter = counter+1;
        end
    end
    if index > 500
        counter = counter+1;
    end
   
    if( le < 0 ) % First phase of landing
		status = 3;
	elseif( le < 2) % Steady descending
		status = 2;
	else % Idle
		status = 1;
	end
end