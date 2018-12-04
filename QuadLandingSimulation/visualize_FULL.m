% Visualize the quadcopter simulation as an animation of a 3D quadcopter.

function visualize_FULL(data)
    grid_rows = 4;
    grid_cols = 6;
    plots = [subplot(grid_rows, grid_cols, [1, 2]),...
		subplot(grid_rows, grid_cols, [7, 8]),...
		subplot(grid_rows, grid_cols, [13, 14]),...
		subplot(grid_rows, grid_cols, [3, 4, 5, 6, 9, 10, 11, 12, 15, 16, 17, 18])]; 


    % Create the quadcopter object
    [t] = quadcopter;
	
    % Animate the quadcopter with data from the simulation.
    animate(data, t, plots);
end

% Animate a quadcopter in flight, using data from the simulation.
function animate(data, model, plots)
    for t = 1:100:length(data.times)
        
        % The first, main part, is for the 3D visualization.
        subplot(plots(4));
        
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
 
        % Update the drawing.      
        xmin = min(data.x(1,:))-20;
        xmax = max(data.x(1,:))+20;
        ymin = min(data.x(2,:))-20;
        ymax = max(data.x(2,:))+20;
        % Z min threshold will be calculated due to the targetpoints which
        % will be on the ground.
		% zmin = min(data.x(3,:))-5;
        zmin = data.targetPoints1(3, 1)-0.5;
		zmax = max(data.x(3,:))+5;

        
       
        tp1 = data.targetPoints1(:, t);
        tp2 = data.targetPoints2(:, t);
        tp3 = data.targetPoints3(:, t);
        tp4 = data.targetPoints4(:, t);
        p1=[tp1(1) tp1(2) tp1(3)];
        p2=[tp2(1) tp2(2) tp2(3)];
        p3=[tp3(1) tp3(2) tp3(3)];
        p4=[tp4(1) tp4(2) tp4(3)];
        p = [p1;p2;p3;p4; p1];

        line(p(:,1), p(:,2), p(:,3), 'LineWidth', 2);
        drawnow;
        title('3D space');
        axis([xmin xmax ymin ymax zmin zmax]);
        xlabel('X axis');
        ylabel('Y axis');
		grid on;
        
        % Calculate max values for the following plots
        max_time = max(data.times);
        
        % Position graph
        subplot(plots(1));
        plot(data.times(1:t), data.x(:, 1:t), 'LineWidth', 2);
	
		% Calculate the min/max values for the axis
		max_x_pos = max(data.x(1, :));
        max_y_pos = max(data.x(2, :));
        max_z_pos = max(data.x(3, :));     
        max_position = max(max_x_pos, max_y_pos);
        max_position = max(max_position, max_z_pos);
        min_x_pos = min(data.x(1, :));
        min_y_pos = min(data.x(2, :));
        min_z_pos = min(data.x(3, :));     
        min_position = min(min_x_pos, min_y_pos);
        min_position = min(min_position, min_z_pos);
		
		% Set some more parameters for the graph
        axis([0 max_time min_position max_position]);
		xlabel('time (sec)')
        ylabel('position (m)')
        %title('Quadcopter position ');
        grid on;

        % Velocity graph
        subplot(plots(2));
        plot(data.times(1:t), data.v(:, 1:t), 'LineWidth', 2);
		
		% Calculate the min/max values for the axis
		min_x_vel = min(data.v(1, :));
        min_y_vel = min(data.v(2, :));
        min_z_vel = min(data.v(3, :));     
        min_velocity = min(min_x_vel, min_y_vel);
        min_velocity = min(min_velocity, min_z_vel);
		max_x_vel = max(data.v(1, :));
        max_y_vel = max(data.v(2, :));
        max_z_vel = max(data.v(3, :));     
        max_velocity = max(max_x_vel, max_y_vel);
        max_velocity = max(max_velocity, max_z_vel);
		
		% Set some more parameters for the graph
        axis([0 max_time min_velocity max_velocity]);
		xlabel('time (sec)')
        ylabel('velocity (m/s^2)')
        %title('Quadcopter velocity ');
        grid on;
% 		
% 		% Annotations for the velocity and position graphs
% 		% Annotation text for the X axis
% 		dim = [0.28 0.88 .08 .024];
% 		str = ' X axis';
% 		annotation('textbox',dim,'String',str);
% 		
% 		% Annotation text for the Y axis
% 		dim = [0.28 0.84 .08 .024];
% 		str = ' Y axis';
% 		annotation('textbox',dim,'String',str);
% 		
% 		% Annotation text for the Z axis
% 		dim = [0.28 0.80 .08 .024];
% 		str = ' Z axis';
% 		annotation('textbox',dim,'String',str);
% 		
% 		% Annotation lines for the X,Y,Z axis respectively
% 		annotation('line', [.32 .35], [.81 .81], 'Color', 'm', 'LineWidth', 2);
% 		annotation('line', [.32 .35], [.85 .85], 'Color', 'r', 'LineWidth', 2);
% 		annotation('line', [.32 .35], [.89 .89], 'Color', 'b', 'LineWidth', 2);
% 
		% Simulation MODE graph
        subplot(plots(3));
		plot(data.times(1:t), data.mymode(:, 1:t), 'LineWidth', 4);
        axis([0 max_time -0.5 3.5]);
		ylabel('MODE')
        xlabel('time (sec)')
		%title('Simulation MODE');
        grid on;
		
% 		% Annotation text for the Y axis
% 		dim = [.76 0.24 .08 .024];
% 		str = '1: DESC mode';
% 		annotation('textbox',dim,'String',str,'Color', 'b');
% 		
% 		% Annotation text for the Z axis
% 		dim = [.76 0.20 .08 .024];
% 		str = '2: Power cut off';
% 		annotation('textbox',dim,'String',str,'Color', 'r');
% 		
% 		% Annotation text for the Z axis
% 		dim = [.76 0.16 .08 .024];
% 		str = '3: Idle mode';
% 		annotation('textbox',dim,'String',str,'Color', 'm');

% 		% On board camera graph
% 		subplot(plots(3));
%        
% 		% Retriece the data from the struct
%         s = data.s;
%         s_des = data.s_des;
% 		
% 		% Plot the desired s projected points in the camera frame.
% 		plot([s_des(1, t) s_des(3, t)], [s_des(2, t) s_des(4, t)], 'Color', 'b', 'LineWidth',2);
% 		hold on;
% 		plot([s_des(3, t) s_des(5, t)], [s_des(4, t) s_des(6, t)], 'Color', 'b', 'LineWidth',2);
% 		hold on;
% 		plot([s_des(5, t) s_des(7, t)], [s_des(6, t) s_des(8, t)], 'Color', 'b', 'LineWidth',2);
% 		hold on;
% 		plot([s_des(7, t) s_des(1, t)], [s_des(8, t) s_des(2, t)], 'Color', 'b', 'LineWidth',2);
% 		hold on;
% 		
% 		% Plot the current s projected points in the camera frame.
% 		plot([s(1, t) s(3, t)], [s(2, t) s(4, t)], 'Color', 'r', 'LineWidth',2);
% 		hold on;
% 		plot([s(3, t) s(5, t)], [s(4, t) s(6, t)], 'Color', 'r', 'LineWidth',2);
% 		hold on;
% 		plot([s(5, t) s(7, t)], [s(6, t) s(8, t)], 'Color', 'r', 'LineWidth',2);
% 		hold on;
% 		plot([s(7, t) s(1, t)], [s(8, t) s(2, t)], 'Color', 'r', 'LineWidth',2);
% 		hold off;
%         
% 		axis([-2 2 -2 2]);
%         %axis equal;
%         xlabel('X (pixels)')
%         ylabel('Y (pixels)')
%         title('Onboard camera');
%         grid on;
     end
end

% Draw a quadcopter. Return a handle to the quadcopter object
% and an array of handles to the thrust display cylinders. 
% These will be transformed during the animation to display
% relative thrust forces.
function [h] = quadcopter()
    % Draw arms.
    h(1) = prism(-5, -0.25, -0.25, 10, 0.5, 0.2);
    h(2) = prism(-0.25, -5, -0.25, 0.5, 10, 0.2);

    % Draw bulbs representing propellers at the end of each arm.
    [x y z] = sphere;
    x = 3.0 * x;
    y = 3.0 * y;
    z = 0.1 * z;
    
    h(3) = surf(x - 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    h(4) = surf(x + 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(5) = surf(x, y - 5, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    h(6) = surf(x, y + 5, z, 'EdgeColor', 'none', 'FaceColor', 'b');

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