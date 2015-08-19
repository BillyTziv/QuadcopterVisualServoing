syms 'phi(t)'
syms 'psi(t)'
syms 'theta(t)'
syms R
R = [cos(psi)*cos(theta)-sin(theta)*sin(phi)*sin(psi) -cos(phi)*sin(psi) cos(psi)*sin(theta) + sin(psi)*sin(phi)*cos(theta);cos(theta) * sin(psi) + cos(psi) * sin(phi)* sin(theta) cos(psi) * cos(phi) sin(psi)*sin(theta) - cos(psi) * sin(phi)*cos(theta);-cos(phi)*sin(theta) sin(phi) cos(phi)*cos(theta)]

Rdot = diff(R, t)
W = Rdot*R'