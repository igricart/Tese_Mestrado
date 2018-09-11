function R_rpy = R_rpy(eta2)
%R_rpy(eta2) computes the RPY rotation matrix R=Rz(psi)Ry(theta)Rx(phi)
phi = eta2(1);
theta = eta2(2);
psi = eta2(3);


sphi = sin(phi);
stheta = sin(theta);
spsi = sin(psi);

cphi = cos(phi);
ctheta = cos(theta);
cpsi = cos(psi);

% R_rpy=Rz(psi)*Ry(theta)*Rx(phi);

R_rpy = [ cpsi*ctheta, cpsi*sphi*stheta - cphi*spsi, sphi*spsi + cphi*cpsi*stheta ;...
          ctheta*spsi, cphi*cpsi + sphi*spsi*stheta, cphi*spsi*stheta - cpsi*sphi ;...
          -stheta, ctheta*sphi, cphi*ctheta ];

end