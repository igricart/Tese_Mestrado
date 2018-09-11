function dT_rpy = dT_rpy(eta2,deta2)
%dT_rpy(eta2,deta2) computes the time derivative of the RPY representation
%Jacobian T_0b(psi,theta,phi)

phi = eta2(1);
theta = eta2(2);
dphi = deta2(1);
dtheta = deta2(2);

sphi = sin(phi);
cphi = cos(phi);
stheta = sin(theta);
ctheta = cos(theta);
ttheta = tan(theta);

dT_rpy = [0 (cphi*ttheta*dphi + sphi/ctheta^2*dtheta) (-sphi*ttheta*dphi + cphi/ctheta^2*dtheta);...
          0 -sphi*dphi -cphi*dphi;...
          0 (cphi/ctheta*dphi + sphi*stheta/ctheta^2*dtheta) (-sphi/ctheta*dphi + cphi*stheta/ctheta^2*dtheta)];

end