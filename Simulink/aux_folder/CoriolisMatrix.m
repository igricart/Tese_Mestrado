function C = CoriolisMatrix_4_joints( q, dq, Mass, Inertia, R, L)
% compite Coriolis matrix for leg prosthesis

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
Theta(1) = 0;%m1 + m2 + m3 + m4;
Theta(2) = C2*m2 + L2*m3 +L2*m4;
Theta(3) = C4*m4;
Theta(4) = C3*m3 + L3*m4;
Theta(5) = 0;%I2 + I3 + I4 + (C2^2)*m2 + (C3^2)*m3 + (C4^2)*m4 + (L2^2)*m3 + (L2^2)*m4 + (L3^2)*m4;
Theta(6) = C4*L2*m4;
Theta(7) = C3*L2*m3 + L2*L3*m4;
Theta(8) = C4*L3*m4;
Theta(9) = 0;%m3*(C3^2) + m4*(C4^2) + m4*(L3^2) + I3 + I4;
Theta(10) = 0;%I4 + (C4^2)*m4;
Theta(11) = 0;%b;
Theta(12) = 0;%f;

% q1 = q(1);
q2 = q(2) + pi/2;
q3 = q(3);
q4 = q(4);

% dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);
dq4 = dq(4);

C = zeros(length(q));

C(1,1) = 0;
C(1,2) = -dq3*(Theta(3)*sin(q2 + q3 + q4) + Theta(4)*sin(q2 + q3)) ...
         -dq2*(Theta(2)*sin(q2) + Theta(3)*sin(q2 + q3 + q4) + Theta(4)*sin(q2 + q3)) ...
         -Theta(3)*dq4*sin(q2 + q3 + q4);
C(1,3) = -dq2*(Theta(3)*sin(q2 + q3 + q4) + Theta(4)*sin(q2 + q3)) ...
         -dq3*(Theta(3)*sin(q2 + q3 + q4) + Theta(4)*sin(q2 + q3)) ...
         -Theta(3)*dq4*sin(q2 + q3 + q4);
C(1,4) = -Theta(3)*dq2*sin(q2 + q3 + q4) -Theta(3)*dq3*sin(q2 + q3 + q4)...
         -Theta(3)*dq4*sin(q2 + q3 + q4);

C(2,1) = 0;
C(2,2) = -dq3*(Theta(6)*sin(q3 + q4) + Theta(7)*sin(q3)) - dq4*(Theta(6)*sin(q3 + q4) + Theta(8)*sin(q4));
C(2,3) = -dq2*(Theta(6)*sin(q3 + q4) + Theta(7)*sin(q3)) - dq3*(Theta(6)*sin(q3 + q4) + Theta(8)*sin(q4)) ...
         -dq4*(Theta(6)*sin(q3 + q4) + Theta(8)*sin(q4));
C(2,4) = -dq2*(Theta(6)*sin(q3 + q4) + Theta(8)*sin(q4)) - dq3*(Theta(6)*sin(q3 + q4) + Theta(8)*sin(q4)) ...
         -dq4*(Theta(6)*sin(q3 + q4) + Theta(8)*sin(q4));

C(3,1) = 0;
C(3,2) = -dq2*(Theta(6)*sin(q3 + q4) + Theta(7)*sin(q3)) - Theta(8)*dq4*sin(q4); 
C(3,3) = -Theta(8)*dq4*sin(q4);
C(3,4) = -Theta(8)*dq2*sin(q4) - Theta(8)*dq3*sin(q4) - Theta(8)*dq4*sin(q4);

C(4,1) = 0;
C(4,2) = dq2*(Theta(6)*sin(q3 + q4) + Theta(8)*sin(q4)) + Theta(8)*dq3*sin(q4);
C(4,3) = Theta(8)*dq2*sin(q4) + Theta(8)*dq3*sin(q4);
C(4,4) = 0;

end

