function [L_1, L_2] = calcJacob(s)
x1_Z = 1;
x2_Z = 1;
x3_Z = 1;
x4_Z = 1;

x_cm1 = s(1);
y_cm1 = s(2);
x_cm2 = s(3);
y_cm2 = s(4);
x_cm3 = s(5);
y_cm3 = s(6);
x_cm4 = s(7);
y_cm4 = s(8);
    
    % Point 1
    Lp1_1 = [-1/x1_Z, 0, x_cm1/x1_Z, y_cm1;0, -1/x1_Z, y_cm1/x1_Z,-x_cm1;];
    Lp1_2 = [x_cm1*y_cm1, -(1+x_cm1^2);1+y_cm1^2, -x_cm1*y_cm1;];
    
    % Point 2
    Lp2_1 = [-1/x2_Z, 0, x_cm2/x2_Z, y_cm2;0, -1/x2_Z, y_cm2/x2_Z,-x_cm2;];
    Lp2_2 = [x_cm2*y_cm2, -(1+x_cm2^2);1+y_cm2^2, -x_cm2*y_cm2;];
    
    % Point 3
    Lp3_1 = [-1/x3_Z, 0, x_cm3/x3_Z, y_cm3;0, -1/x3_Z, y_cm3/x3_Z, -x_cm3;];
    Lp3_2 = [ x_cm3*y_cm3, -(1+x_cm3^2);1+y_cm3^2, -x_cm3*y_cm3];
    
    % Point 4
    Lp4_1 = [-1/x4_Z, 0, x_cm4/x4_Z, y_cm4;0, -1/x4_Z, y_cm4/x4_Z,-x_cm4;];
    Lp4_2 = [x_cm4*y_cm4, -(1+x_cm4^2);1+y_cm4^2, -x_cm4*y_cm4;];
    
    % For all the four points the following vector contains the vx, vy, vz,
    % omegaz. 
    L_1 =  [Lp1_1; Lp2_1; Lp3_1; Lp4_1;];
    
    % For all the four points the following vector contains omegax, omegay.
    L_2 =  [Lp1_2; Lp2_2; Lp3_2; Lp4_2;];
end