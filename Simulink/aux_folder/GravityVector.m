function G = GravityVector( q, Mass, Inertia, R, L, g)

I2 = Inertia(2,3);
I3 = Inertia(2,4);
I4 = Inertia(2,5);

C2 = R(3,3);
C3 = R(3,4);
C4 = R(3,5);

L2 = L(3,3);
L3 = L(3,4);
L4 = 0.1;


m1 = Mass(2);
m2 = Mass(3);
m3 = Mass(4);
m4 = Mass(5);

%% Theta matrix (parameters)
Theta1 = m1 + m2 + m3 + m4;
Theta2 = C2*m2 + L2*m3 +L2*m4;
Theta3 = C4*m4;
Theta4 = C3*m3 + L3*m4;
Theta5 = I2 + I3 + I4 + (C2^2)*m2 + (C3^2)*m3 + (C4^2)*m4 + (L2^2)*m3 + (L2^2)*m4 + (L3^2)*m4;
Theta6 = C4*L2*m4;
Theta7 = C3*L2*m3 + L2*L3*m4;
Theta8 = C4*L3*m4;
Theta9 = m3*(C3^2) + m4*(C4^2) + m4*(L3^2) + I3 + I4;
Theta10 = I4 + (C4^2)*m4;

Theta1 = ones(size(q,1),1) .* Theta1;
Theta2 = ones(size(q,1),1) .* Theta2;
Theta3 = ones(size(q,1),1) .* Theta3;
Theta4 = ones(size(q,1),1) .* Theta4;
Theta5 = ones(size(q,1),1) .* Theta5;
Theta6 = ones(size(q,1),1) .* Theta6;
Theta7 = ones(size(q,1),1) .* Theta7;
Theta8 = ones(size(q,1),1) .* Theta8;
Theta9 = ones(size(q,1),1) .* Theta9;
Theta10 = ones(size(q,1),1) .* Theta10;


q1 = q(:,1);
q2 = q(:,2) + pi/2;
q3 = q(:,3);
q4 = q(:,4);

G1 = -g.*Theta1;
G2 = -g.*(Theta2.*cos(q2) + Theta3.*cos(q2 + q3 + q4) + Theta4.*cos(q2 + q3));
G3 = -g.*(Theta4.*cos(q2+q3) + Theta3.*cos(q2 + q3 + q4));
G4 = -g.*Theta3.*cos(q2 + q3 + q4);

G1 = reshape(G1,1,1,size(q,1));
G2 = reshape(G2,1,1,size(q,1));
G3 = reshape(G3,1,1,size(q,1));
G4 = reshape(G4,1,1,size(q,1));

G = [G1, G2, G3, G4];

end