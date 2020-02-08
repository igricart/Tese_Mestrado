function Jvt = JacobianToeMatrix_4_joints( q, L2, L3, ah, aT)
% compute the linear velocity Jacobians considering heel as contact point

% q1 = q(1);
q2 = q(2) + pi/2;
q3 = q(3);
q4 = q(4);


Jvt(1,1) = 0;
Jvt(1,2) = -L2*sin(q2) - L3*sin(q2+q3) - aT*sin(q2 + q3 + q4 + (pi/2 - acos(ah/aT)));
Jvt(1,3) = -L3*sin(q2 + q3) - aT*sin(q2 + q3 + q4 + (pi/2 - acos(ah/aT)));
Jvt(1,4) = -aT*sin(q2 + q3 + q4 + (pi/2 - acos(ah/aT)));

Jvt(2,1) = 1;
Jvt(2,2) = L2*cos(q2) + L3*cos(q2 + q3) + aT*cos(q2 + q3 + q4 + (pi/2 - acos(ah/aT)));
Jvt(2,3) = L3*cos(q2 + q3) + aT*cos(q2 + q3 + q4 + (pi/2 - acos(ah/aT)));
Jvt(2,4) = aT*cos(q2+q3+q4 + (pi/2 - acos(ah/aT)));

end
