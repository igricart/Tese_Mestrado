function M = MassMatrix( q, Mass, Inertia, R, L)
% compute mass matrix for leg prosthesis

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
Theta(5) = I2 + I3 + I4 + (C2^2)*m2 + (C3^2)*m3 + (C4^2)*m4 + (L2^2)*m3 + (L2^2)*m4 + (L3^2)*m4;
Theta(6) = C4*L2*m4;
Theta(7) = C3*L2*m3 + L2*L3*m4;
Theta(8) = C4*L3*m4;
Theta(9) = m3*(C3^2) + m4*(C4^2) + m4*(L3^2) + I3 + I4;
Theta(10) = I4 + (C4^2)*m4;

% q1 = q(1);
q2 = q(2) + pi/2;
q3 = q(3);
q4 = q(4);

M = zeros(length(q));

M(1,1) = Theta(1);
M(1,2) = Theta(2)*cos(q2) + Theta(3)*cos(q2 + q3 + q4) + Theta(4)*cos(q2 + q3);
M(1,3) = Theta(3)*cos(q2 + q3 + q4) + Theta(4)*cos(q2 + q3);
M(1,4) = Theta(3)*cos(q2 + q3 + q4);

M(2,1) = M(1,2);
M(2,2) = Theta(5) + 2*Theta(6)*cos(q3 + q4) + 2*Theta(7)*cos(q3) + 2*Theta(8)*cos(q4);
M(2,3) = Theta(9) + Theta(7)*cos(q3) + 2*Theta(8)*cos(q4) + Theta(6)*cos(q3 + q4);
M(2,4) = Theta(10) + Theta(6)*cos(q3 + q4) + Theta(8)*cos(q4);

M(3,1) = M(1,3); 
M(3,2) = M(2,3);
M(3,3) = Theta(9) + 2*Theta(8)*cos(q4) + m4*L3^2 + I3 + I4;
M(3,4) = Theta(10) + Theta(8)*cos(q4)

M(4,1) = M(1,4);
M(4,2) = M(2,4);
M(4,3) = M(3,4);
M(4,4) = Theta(10);

end

