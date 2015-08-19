% Visualize the quadcopter simulation as an animation of a 3D quadcopter.

function visualize(data)
    % Create a figure with three parts. One part is for a 3D visualization,
    % and the other two are for running graphs of angular velocity and displacement.
 
   
    plots = [subplot(4, 6, [1, 2]), subplot(4, 6, [7, 8]), subplot(4, 6, [21, 22, 23, 24]),subplot(4, 6, [19, 20]), subplot(4, 6, [3, 4, 5, 6, 9, 10, 11, 12, 15, 16, 17, 18]), subplot(4, 6, [13, 14]),];
    subplot(plots(5));

    % Create the quadcopter object. Returns a handle to
    % the quadcopter itself as well as the thrust-display cylinders.
    [t, thrusts] = quadcopter;

    % Set axis scale and labels.

    max_height = max(data.x(3, :));
    axis([-50 50 -50 50 0 max_height]);
    grid on;
    
    % Animate the quadcopter with data from the simulation.
    animate(data, t, thrusts, plots);
    
end

% Animate a quadcopter in flight, using data from the simulation.
function animate(data, model, thrusts, plots)
    
    % Show frames from the animation. However, in the interest of speed,
    % skip some frames to make the animation more visually appealing.
   
    for t = 1:10:length(data.times)
    
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
        scales = exp(data.input(:, t) / min(abs(data.input(:, t))) + 5) - exp(6) +  1.5;
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
            set(thrusts(i), 'Matrix', move * rotate * scalez);
        end
     
        % Update the drawing.      
        xmin = min(data.x(1,:))-20;
        xmax = max(data.x(1,:))+20;
        ymin = min(data.x(2,:))-20;
        ymax = max(data.x(2,:))+20;
        zmin = min(data.x(3,:));
        zmax = max(data.x(3,:))+5;
      
        axis([xmin xmax ymin ymax zmin zmax]);
        drawnow;
        
        % Draw an X at the ground...
        v1 = [xmin:xmax];
        v2 = [ymin:ymax];
        v3 = fliplr(v1);
        v4 = [ymin:ymax];
        plot(v1, v2, 'b', v3, v4, 'b', 'LineWidth', 1);
        
        plot(data.x(1, :), data.x(2, :), 'LineWidth', 2);
        
        
        % Quadcopter height graph
        subplot(plots(1));
        max_height = max(data.x(3, :));
        max_time = max(data.times);
        plot(data.times(1:t), data.x(3, 1:t), 'Color', 'b', 'LineWidth', 2);
        axis([0 max_time 0 max_height]);
        title('Height');
        grid on;
%         xlabel('Time (s)');
%         ylabel('Height (m)');
%         title('Height');
 
        % Quadcopter theta angles (phi, psi, theta) graph
        subplot(plots(2));
        plot(data.times, data.theta(1, :), 'Color', 'r', 'LineWidth', 2); hold on;
        plot(data.times, data.theta(2, :), 'Color', 'g', 'LineWidth', 2); hold on;
        plot(data.times, data.theta(3, :), 'Color', 'y', 'LineWidth', 2); hold off;
        title('Angles');
         grid on;
%         xlabel('Time (s)');
%         ylabel('Theta(rad)');
%         title('Theta phi(r) psi(g) theta(y)');
%         
        % Engine RPM bar graph
        subplot(plots(3));
        max_thrust = max(max(data.thrust));
        bar(data.thrust(1:3, t), 'y', 'LineWidth', 2);
        %ylabel('Engine RPM');
        ylim([0 max_thrust])
        title('Thrust on X, Y, Z axis');
       
        
        subplot(plots(6));
        max_error = max(data.F_des(1, :));
        max_time = max(data.times);
        plot(data.times(1:t), data.F_des(1:t), 'Color', 'g', 'LineWidth', 2);
        axis([0 max_time 0 max_error]);
        title('Distance error from desired position');
         grid on;
        % Show logo image
%         subplot(plots(1));
%         im = imread('logo.png');
%         imshow(im);

   
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
    thrusts(1) = surf(x, y + 5, z, 'EdgeColor', 'none', 'FaceColor', 'k');
    thrusts(2) = surf(x + 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'k');
    thrusts(3) = surf(x, y - 5, z, 'EdgeColor', 'none', 'FaceColor', 'k');
    thrusts(4) = surf(x - 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'k');

    % Create handles for each of the thrust cylinders.
    for i = 1:4
        x = hgtransform;
        set(thrusts(i), 'Parent', x);
        thrusts(i) = x;
    end

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