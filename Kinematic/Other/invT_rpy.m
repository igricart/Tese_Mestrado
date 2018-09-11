function invT = invT_rpy(eta2)
%invTb(eta2) is the inverse of the RPY represenation Jacobian T(eta2),
%eta2=[phi;theta;psi] is the RPY angles

phi = eta2(1);
theta = eta2(2);

sphi = sin(phi);
cphi = cos(phi);
stheta = sin(theta);
ctheta = cos(theta);


invT = [1 0 -stheta;...
        0 cphi ctheta*sphi;...
        0 -sphi ctheta*cphi];

end


