function C = CoriolisMatrix( q, dq, Mass, Inertia, R, L)
% compute Coriolis matrix for leg prosthesis

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

dq1 = dq(:,1);
dq2 = dq(:,2);
dq3 = dq(:,3);
dq4 = dq(:,4);


C11 = ones(size(q,1),1) .* 0;
C12 = -dq3.*(Theta3.*sin(q2 + q3 + q4) + Theta4.*sin(q2 + q3)) ...
         -dq2.*(Theta2.*sin(q2) + Theta3.*sin(q2 + q3 + q4) + Theta4.*sin(q2 + q3)) ...
         -Theta3.*dq4.*sin(q2 + q3 + q4);
C13 = -dq2.*(Theta3.*sin(q2 + q3 + q4) + Theta4.*sin(q2 + q3)) ...
         -dq3.*(Theta3.*sin(q2 + q3 + q4) + Theta4.*sin(q2 + q3)) ...
         -Theta3.*dq4.*sin(q2 + q3 + q4);
C14 = -Theta3.*dq2.*sin(q2 + q3 + q4) -Theta3.*dq3.*sin(q2 + q3 + q4)...
         -Theta3.*dq4.*sin(q2 + q3 + q4);

C21 = ones(size(q,1),1) .* 0;
C22 = -dq3.*(Theta6.*sin(q3 + q4) + Theta7.*sin(q3)) - dq4.*(Theta6.*sin(q3 + q4) + Theta8.*sin(q4));
C23 = -dq2.*(Theta6.*sin(q3 + q4) + Theta7.*sin(q3)) - dq3.*(Theta6.*sin(q3 + q4) + Theta8.*sin(q4)) ...
         -dq4.*(Theta6.*sin(q3 + q4) + Theta8.*sin(q4));
C24 = -dq2.*(Theta6.*sin(q3 + q4) + Theta8.*sin(q4)) - dq3.*(Theta6.*sin(q3 + q4) + Theta8.*sin(q4)) ...
         -dq4.*(Theta6.*sin(q3 + q4) + Theta8.*sin(q4));

C31 = ones(size(q,1),1) .* 0;
C32 = -dq2.*(Theta6.*sin(q3 + q4) + Theta7.*sin(q3)) - Theta8.*dq4.*sin(q4);
C33 = -Theta8.*dq4.*sin(q4);
C34 = -Theta8.*dq2.*sin(q4) - Theta8.*dq3.*sin(q4) - Theta8.*dq4.*sin(q4);

C41 = ones(size(q,1),1) .* 0;
C42 = dq2.*(Theta6.*sin(q3 + q4) + Theta8.*sin(q4)) + Theta8.*dq3.*sin(q4);
C43 = Theta8.*dq2.*sin(q4) + Theta8.*dq3.*sin(q4);
C44 = ones(size(q,1),1) .* 0;

C11 = reshape(C11,1,1,size(q,1));
C12 = reshape(C12,1,1,size(q,1));
C13 = reshape(C13,1,1,size(q,1));
C14 = reshape(C14,1,1,size(q,1));
C21 = reshape(C21,1,1,size(q,1));
C22 = reshape(C22,1,1,size(q,1));
C23 = reshape(C23,1,1,size(q,1));
C24 = reshape(C24,1,1,size(q,1));
C31 = reshape(C31,1,1,size(q,1));
C32 = reshape(C32,1,1,size(q,1));
C33 = reshape(C33,1,1,size(q,1));
C34 = reshape(C34,1,1,size(q,1));
C41 = reshape(C41,1,1,size(q,1));
C42 = reshape(C42,1,1,size(q,1));
C43 = reshape(C43,1,1,size(q,1));
C44 = reshape(C44,1,1,size(q,1));

C = [C11 C12 C13 C13; C21 C22 C23 C24; C31 C32 C33 C34; C41 C42 C43 C44];

end