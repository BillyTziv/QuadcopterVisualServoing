% Calculates the inertia matrix according the function arguments. Notice
% that all the above calculations are based on an <X> shape quadcopter
% Input and output are on the SI system.
function I = inertiaMatrix(MotorMass, RodMass, RodLength)
    % I = RodLength*RodLength* (RodMass + MotorMass);
    I = diag([5e-3, 5e-3, 10e-3]);  % moment of inertia
end