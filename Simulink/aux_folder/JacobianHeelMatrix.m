function Jvh = JacobianHeelMatrix_4_joints( q, L2, L3, ah, aH)
% compute the linear velocity Jacobians considering heel as contact point

% q1 = q(1);
q2 = q(2) + pi/2;
q3 = q(3);
q4 = q(4);


Jvh(1,1) = 0;
Jvh(1,2) = -L2*sin(q2) - L3*sin(q2 + q3) - aH*sin(q2 + q3 + q4 + (pi/2 + acos(ah/aH)));
Jvh(1,3) = -L3*sin(q2 + q3) - aH*sin(q2 + q3 + q4 + (pi/2 + acos(ah/aH)));
Jvh(1,4) = -aH*sin(q2 + q3 + q4 + (pi/2 + acos(ah/aH)));

Jvh(2,1) = 1;
Jvh(2,2) = L2*cos(q2) + L3*cos(q2 + q3) + aH*cos(q2 + q3 + q4 + (pi/2 + acos(ah/aH)));
Jvh(2,3) = L3*cos(q2 + q3) + aH*cos(q2 + q3 + q4 + (pi/2 + acos(ah/aH)));
Jvh(2,4) = aH*cos(q2 + q3 + q4 + (pi/2 + acos(ah/aH)));

end

