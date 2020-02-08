function G = GravityVector( q, Mass, Inertia, R, L, g)

I2 = Inertia(2,3);
I3 = Inertia(2,4);
I4 = Inertia(2,5);

C2 = R(3,3);
C3 = R(3,4);
C4 = R(3,5);

L2 = L(2,3);
L3 = L(2,4);
L4 = 0.1;


m1 = Mass(2);
m2 = Mass(3);
m3 = Mass(4);
m4 = Mass(5);

%% Theta matrix (parameters)
Theta(1) = m1 + m2 + m3 + m4;
Theta(2) = C2*m2 + L2*m3 +L2*m4;
Theta(3) = C4*m4;
Theta(4) = C3*m3 + L3*m4;

% q1 = q(1);
q2 = q(2) + pi/2;
q3 = q(3);
q4 = q(4);

G = [ -g*Theta(1);
      -g*(Theta(2)*cos(q2) + Theta(3)*cos(q2 + q3 + q4) + Theta(4)*cos(q2 + q3));
      -g*Theta(4)*cos(q2+q3)-g*Theta(3)*cos(q2 + q3 + q4);
      -g*Theta(3)*cos(q2 + q3 + q4)];

end