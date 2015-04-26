function a = acceleration(phi, psi, theta, k, mass, weight, inputs)
    R = rotation(psi, theta, phi);
    thrust = [0; 0; k * sum(inputs)];
    a = weight + R*thrust/mass;
end