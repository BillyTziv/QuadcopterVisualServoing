function output = applyVoltageConstrains(v_mot, v_mot_max)
        % Set motor voltage constrains (LOW and HIGH limit)
        if( v_mot(1)<0)
            v_mot(1)=0;
        end
        if( v_mot(2)<0)
            v_mot(2)=0;
        end
        if( v_mot(3)<0)
            v_mot(3)=0;
        end
        if( v_mot(4)<0)
            v_mot(4)=0;
        end
        if(v_mot(1)>v_mot_max)
            v_mot(1)=v_mot_max;
        end
        if(v_mot(2)>v_mot_max)
            v_mot(2)=v_mot_max;
        end
        if(v_mot(3)>v_mot_max)
            v_mot(3)=v_mot_max;
        end
        if(v_mot(4)>v_mot_max)
            v_mot(4)=v_mot_max;
		end
		
		output = v_mot;
end