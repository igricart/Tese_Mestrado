function dinvT_rpy = dinvT_rpy(eta2,deta2)
%dinvT_rpy(eta2,deta2) computes the time derivative of the inverse of RPY
%representation Jacobian T_0b(psi,theta,phi)

phi = eta2(1);
theta = eta2(2);
dphi = deta2(1);
dtheta = deta2(2);

stheta = sin(theta);
ctheta = cos(theta);
sphi = sin(phi);
cphi = cos(phi);

dinvT_rpy = [0 0 -ctheta*dtheta;...
             0 -sphi*dphi (cphi*ctheta*dphi - sphi*ctheta*dtheta);...
             0 -cphi*dphi (-sphi*ctheta*dphi + cphi*stheta*dtheta)];

end