% Visualize the quadcopter simulation as an animation of a 3D quadcopter.

function visualize(data)
    grid_rows = 4;
    grid_cols = 6;
    plots = [subplot(grid_rows, grid_cols, [1, 2]), subplot(grid_rows, grid_cols, [7, 8]), subplot(grid_rows, grid_cols, [13, 14]), subplot(grid_rows, grid_cols, [19, 20]), subplot(grid_rows, grid_cols, [3, 4, 5, 6, 9, 10, 11, 12, 15, 16, 17, 18]), subplot(grid_rows, grid_cols, [21, 22, 23, 24]) ];
             
    subplot(plots(5));

    % Create the quadcopter object. Returns a handle to
    % the quadcopter itself as well as the thrust-display cylinders.
    [t, thrusts] = quadcopter;

    % Set axis scale and labels.
    max_height = max(data.x(3, :));
    %axis([-50 50 -50 50 0   max_height]);
    axis([-50 50 -50 50 0   max_height+10]);% kw
    grid on;
    
    % Animate the quadcopter with data from the simulation.
    animate(data, t, thrusts, plots);
end

% Animate a quadcopter in flight, using data from the simulation.
function animate(data, model, thrusts, plots)
    
    % Show frames from the animation. However, in the interest of speed,
    % skip some frames to make the animation more visually appealing.
   
    %for t = 1:10:length(data.times)
    for t = 1:10:length(data.times)% kw
        
        % The first, main part, is for the 3D visualization.
        subplot(plots(5));
        
        % Compute translation to correct linear position coordinates.
        dx = data.x(:, t);
        
        move = makehgtform('translate', dx);

        % Compute rotation to correct angles. Then, turn this rotation
        % into a 4x4 matrix represting this affine transformation.
        angles = data.theta(:, t);
        rotate = rotation(angles);
        rotate = [rotate zeros(3, 1); zeros(1, 3) 1];
       
        
        % Move the quadcopter to the right place, after putting it in the correct orientation.
        set(model,'Matrix', move * rotate);
  
        % Compute scaling for the thrust cylinders. The lengths should represent relative
        % strength of the thrust at each propeller, and this is just a heuristic that seems
        % to give a good visual indication of thrusts.
        scales = exp(data.eng_RPM(:, t) / min(abs(data.eng_RPM(:, t))) + 5) - exp(6) +  1.5;
        for i = 1:4
      
            % Scale each cylinder. For negative scales, we need to flip the cylinder
            % using a rotation, because makehgtform does not understand negative scaling.
            s = scales(i);
            if s < 0
                scalez = makehgtform('yrotate', pi)  * makehgtform('scale', [1, 1, abs(s)]);
            elseif s > 0
                scalez = makehgtform('scale', [1, 1, s]);
            end

            % Scale the cylinder as appropriate, then move it to
            % be at the same place as the quadcopter propeller.
%             set(thrusts(i), 'Matrix', move * rotate * scalez);
        end
     
        % Update the drawing.      
        xmin = min(data.x(1,:))-20;
        xmax = max(data.x(1,:))+20;
        ymin = min(data.x(2,:))-20;
        ymax = max(data.x(2,:))+20;
        zmin = min(data.x(3,:));
        zmax = max(data.x(3,:))+5;
      
        axis([xmin xmax ymin ymax zmin zmax]);
        xlabel('X axis');
        ylabel('Y axis');
        drawnow;
        
        % Calculate max values for the following plots
        max_time = max(data.times);
        
        max_x_pos = max(data.x(1, :));
        max_y_pos = max(data.x(2, :));
        max_z_pos = max(data.x(3, :));     
        max_position = max(max_x_pos, max_y_pos);
        max_position = max(max_position, max_z_pos);
        
        max_x_vel = max(data.v(1, :));
        max_y_vel = max(data.v(2, :));
        max_z_vel = max(data.v(3, :));     
        max_velocity = max(max_x_vel, max_y_vel);
        max_velocity = max(max_velocity, max_z_vel);
        
        max_x_ace = max(data.a(1, :));
        max_y_ace = max(data.a(2, :));
        max_z_ace = max(data.a(3, :));     
        max_acceleration = max(max_x_ace, max_y_ace);
        max_acceleration = max(max_acceleration, max_z_ace);
        
        % Calculate min values for the following plots
        min_x_pos = min(data.x(1, :));
        min_y_pos = min(data.x(2, :));
        min_z_pos = min(data.x(3, :));     
        min_position = min(min_x_pos, min_y_pos);
        min_position = min(min_position, min_z_pos);
        
        min_x_vel = min(data.v(1, :));
        min_y_vel = min(data.v(2, :));
        min_z_vel = min(data.v(3, :));     
        min_velocity = min(min_x_vel, min_y_vel);
        min_velocity = min(min_velocity, min_z_vel);
        
        min_x_ace = min(data.a(1, :));
        min_y_ace = min(data.a(2, :));
        min_z_ace = min(data.a(3, :));     
        min_acceleration = min(min_x_ace, min_y_ace);
        min_acceleration = min(min_acceleration, min_z_ace);
        
        % Calculate max torque for the following plots
        max_x_torque = max(data.torque(1, :));
        max_y_torque = max(data.torque(2, :));
        max_z_torque = max(data.torque(3, :));      
        max_torque = max(max_x_torque, max_y_torque);
        max_torque = max(max_torque, max_z_torque);
        
        % Calculate min torque for the following plots
        min_x_torque = min(data.torque(1, :));
        min_y_torque = min(data.torque(2, :));
        min_z_torque = min(data.torque(3, :));      
        min_torque = min(min_x_torque, min_y_torque);
        min_torque = min(min_torque, min_z_torque);
        
        % Calculate max engine RPM
        max_eng1_RPM = max(data.eng_RPM(1, :));
        max_eng2_RPM = max(data.eng_RPM(2, :));
        max_eng3_RPM = max(data.eng_RPM(3, :));
        max_eng4_RPM = max(data.eng_RPM(4, :));
        max_front_eng_RPM = max(max_eng1_RPM, max_eng2_RPM);
        max_rear_eng_RPM = max(max_eng3_RPM, max_eng4_RPM);
        max_eng_RPM = max(max_front_eng_RPM, max_rear_eng_RPM);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plot the position of the vehicle
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(plots(1));
       
        plot(data.times(1:t), data.x(:, 1:t), 'LineWidth', 2);
        axis([0 max_time min_position max_position]);
        title('Position');
        grid on;
     
       
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plot the velocity of the vehicle
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(plots(2));
        
        plot(data.times(1:t), data.v(:, 1:t), 'LineWidth', 2);
        axis([0 max_time min_velocity max_velocity]);
        title('Velocity');
        grid on;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plot the acceleration of the vehicle
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(plots(3));
        
        plot(data.times(1:t), data.a(:, 1:t), 'LineWidth', 2);
        axis([0 max_time min_acceleration max_acceleration]);
        title('Acceleration');
        grid on;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plot the torques of the vehicle
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(plots(4));
        
        plot(data.times(1:t), data.torque(:, 1:t), 'LineWidth', 2);
        axis([0 max_time min_torque max_torque]);
        title('Torque');
        grid on;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plot in bar the engine RPMs of the vehicle
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        subplot(plots(6));
        
        bar(data.eng_RPM(1:4, t), 'g', 'LineWidth', 2);
        ylim([0 max_eng_RPM])
        title('Engine RPM');
    end
end

% Plot three components of a vector in RGB.
function multiplot(data, values, ind)
    % Select the parts of the data to plot.
    times = data.times(:, 1:ind);
    values = values(:, 1:ind);

    % Plot in RGB, with different markers for different components.
    plot(times, values(1, :), 'r-', times, values(2, :), 'g.', times, values(3, :), 'b-.');
    
    % Set axes to remain constant throughout plotting.
    xmin = min(data.times);
    xmax = max(data.times);
    ymin = 1.1 * min(min(values));
    ymax = 1.1 * max(max(values));
    axis([xmin xmax ymin ymax]);
end

% Draw a quadcopter. Return a handle to the quadcopter object
% and an array of handles to the thrust display cylinders. 
% These will be transformed during the animation to display
% relative thrust forces.
function [h, thrusts] = quadcopter()
    % Draw arms.
    h(1) = prism(-5, -0.25, -0.25, 10, 0.5, 0.5);
    h(2) = prism(-0.25, -5, -0.25, 0.5, 10, 0.5);

    % Draw bulbs representing propellers at the end of each arm.
    [x y z] = sphere;
    x = 3.0 * x;
    y = 3.0 * y;
    z = 0.1 * z;
    
    h(3) = surf(x - 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'g');
    h(4) = surf(x + 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(5) = surf(x, y - 5, z, 'EdgeColor', 'none', 'FaceColor', 'g');
    h(6) = surf(x, y + 5, z, 'EdgeColor', 'none', 'FaceColor', 'b');

    % Draw thrust cylinders.
    [x y z] = cylinder(0.1, 7);
    thrusts(1) = surf(x, y, z, 'EdgeColor', 'none', 'FaceColor', 'k');
    thrusts(2) = surf(x, y, z, 'EdgeColor', 'none', 'FaceColor', 'k');
    thrusts(3) = surf(x, y, z, 'EdgeColor', 'none', 'FaceColor', 'k');
    thrusts(4) = surf(x, y, z, 'EdgeColor', 'none', 'FaceColor', 'k');

    % Create handles for each of the thrust cylinders.
%     for i = 1:4
%         x = hgtransform;
%         set(thrusts(i), 'Parent', x);
%         thrusts(i) = x;
%     end

    % Conjoin all quadcopter parts into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Draw a 3D prism at (x, y, z) with width w,
% length l, and height h. Return a handle to
% the prism object.
function h = prism(x, y, z, w, l, h)
    [X Y Z] = prism_faces(x, y, z, w, l, h);

    faces(1, :) = [4 2 1 3];
    faces(2, :) = [4 2 1 3] + 4;
    faces(3, :) = [4 2 6 8];
    faces(4, :) = [4 2 6 8] - 1;
    faces(5, :) = [1 2 6 5];
    faces(6, :) = [1 2 6 5] + 2;

    for i = 1:size(faces, 1)
        h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); hold on;
    end

    % Conjoin all prism faces into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Compute the points on the edge of a prism at
% location (x, y, z) with width w, length l, and height h.
function [X Y Z] = prism_faces(x, y, z, w, l, h)
    X = [x x x x x+w x+w x+w x+w];
    Y = [y y y+l y+l y y y+l y+l];
    Z = [z z+h z z+h z z+h z z+h];
end