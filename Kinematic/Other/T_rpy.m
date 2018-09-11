function T_rpy = T_rpy(eta2)
% T_rpy(eta2) computes the RPY velocity transformation matrix T(eta2)
phi=eta2(1);
theta=eta2(2);

sphi = sin(phi);
cphi = cos(phi);
ttheta = tan(theta);
ctheta = cos(theta);

T_rpy = [1 sphi*ttheta cphi*ttheta;...
         0 cphi -sphi;...
         0 sphi/ctheta cphi/ctheta];

end