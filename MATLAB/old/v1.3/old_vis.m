% TODO add angles for a full animation

% This function creates a 3D quadcopter at postion xin, yin, zin and angles
% phi, psi, theta.
function old_vis(xin, yin, zin, x_out, v_out, a_out, times, index, endTime, F_des, eng_omega)
    subplot(4,3,1)
   
    grid off;
    plot(times(1:index), x_out(3, 1:index), 'b', 'LineWidth', 2);
    ylabel('Altitude');
    axis([0 endTime min(x_out(3, :)) max(x_out(3, :))]);
    
    subplot(4,3,4)
    grid off;
    plot(times(1:index), F_des(1, 1:index), 'r', 'LineWidth', 2);
    ylabel('Error');
    axis([0 endTime min(F_des(1, :)) max(F_des(1, :))]);
    
    subplot(4,3,7)
    grid off;
    bar(eng_omega(1:4, index), 'g', 'LineWidth', 2);
    ylabel('Engine RPM');
    ylim([0 max(max(eng_omega))])
    
    subplot(4,3,[2,3,5,6,8,9])
    % Sphere Radius - details
    %sphere_radius = 50;
    
    % Create a sphere for the motor 1
    [X1, Y1, Z1] = sphere;
    X1 = 0.5*X1;
    Y1 = 0.5*Y1;
    Z1 = 0.5*Z1;

    % Create a sphere for the motor 2
    %[X2, Y2, Z2] = sphere(sphere_radius);
    
    % Create a sphere for the motor 3
    %[X3, Y3, Z3] = sphere(sphere_radius);
    
    % Create a sphere for the motor 4
    %[X4, Y4, Z4] = sphere(sphere_radius);
    
    % Coordinates of the four motors
    M1 = [xin-2.5, yin-2.5, zin];
    M2 = [xin-2.5, yin+2.5, zin];
    M3 = [xin+2.5, yin-2.5, zin];
    M4 = [xin+2.5, yin+2.5, zin];
    
    % Mesh for teh motor 1
    mesh(X1+M1(1), Y1+M1(2), Z1+M1(3));
    hold on;
    
    % Mesh for teh motor 1
    mesh(X1+M2(1), Y1+M2(2), Z1+M2(3));
    hold on;
    
    % Mesh for teh motor 1
    mesh(X1+M3(1), Y1+M3(2), Z1+M3(3));
    hold on;
    
    % Mesh for teh motor 1
    mesh(X1+M4(1), Y1+M4(2), Z1+M4(3));
    hold on;
    
    % Front left rod
    flrx = xin-2.5 : .1 : xin;
    flry = yin-2.5 : .1 : yin;
    flrz = zeros(1, numel(flrx));
    flrz(:) = zin;
    
    % Front right rod
    frrx = xin : .1 : xin+2.5;
    frry = yin : .1 : yin+2.5;
    frrz = zeros(1, numel(frrx));
    frrz(:) = zin;
    
    % Back right rod
    brrx = xin : .1 : xin+2.5;
    brry = fliplr(yin-2.5 : .1 : yin);
    brrz = zeros(1, numel(brrx));
    brrz(:) = zin;
    
    % Back left rod
    blrx = xin-2.5 : .1 : xin;
    blry = fliplr(yin : .1 : yin+2.5);
    blrz = zeros(1, numel(brrx));
    blrz(:) = zin;
    
    axis([-40 40 -40 40 -10 50]);
    xlabel('');
    ylabel('');
    zlabel('Height');
    grid on;
    colormap(bone);
    title('Simulating the position of a quadcopter [RedR: Front View]');
    
    plot3(flrx, flry, flrz, 'b', frrx, frry, frrz, 'r', brrx, brry, brrz, 'b', blrx, blry, blrz, 'r', 'LineWidth', 5)
end