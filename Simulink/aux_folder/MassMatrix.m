function M = MassMatrix( q, Mass, Inertia, R, L)
% compute mass matrix for leg prosthesis

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

M11 = Theta1;
M12 = Theta2.*cos(q2) + Theta3.*cos(q2 + q3 + q4) + Theta4.*cos(q2 + q3);
M13 = Theta3.*cos(q2 + q3 + q4) + Theta4.*cos(q2 + q3);
M14 = Theta3.*cos(q2 + q3 + q4);

M21 = M12;
M22 = Theta5 + 2.*Theta6.*cos(q3 + q4) + 2.*Theta7.*cos(q3) + 2.*Theta8.*cos(q4);
M23 = Theta9 + Theta7.*cos(q3) + 2.*Theta8.*cos(q4) + Theta6.*cos(q3 + q4);
M24 = Theta10 + Theta6.*cos(q3 + q4) + Theta8.*cos(q4);

M31 = M13;
M32 = M23;
M33 = Theta9 + 2.*Theta8.*cos(q4) + m4.*L3^2 + I3 + I4;
M34 = Theta10 + Theta8.*cos(q4);

M41 = M14;
M42 = M24;
M43 = M34;
M44 = Theta10;

M11 = reshape(M11,1,1,size(q,1));
M12 = reshape(M12,1,1,size(q,1));
M13 = reshape(M13,1,1,size(q,1));
M14 = reshape(M14,1,1,size(q,1));
M21 = reshape(M21,1,1,size(q,1));
M22 = reshape(M22,1,1,size(q,1));
M23 = reshape(M23,1,1,size(q,1));
M24 = reshape(M24,1,1,size(q,1));
M31 = reshape(M31,1,1,size(q,1));
M32 = reshape(M32,1,1,size(q,1));
M33 = reshape(M33,1,1,size(q,1));
M34 = reshape(M34,1,1,size(q,1));
M41 = reshape(M41,1,1,size(q,1));
M42 = reshape(M42,1,1,size(q,1));
M43 = reshape(M43,1,1,size(q,1));
M44 = reshape(M44,1,1,size(q,1));

M = [M11 M12 M13 M13; M21 M22 M23 M24; M31 M32 M33 M34; M41 M42 M43 M44];

end

