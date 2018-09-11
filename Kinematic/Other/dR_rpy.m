function dR_rpy = dR_rpy(eta2,deta2)
%dR_rpy(eta2,deta2) computes the time derivative of the RPY rotation matrix R=Rz(psi)Ry(theta)Rx(phi)

phi = eta2(1);
theta = eta2(2);
psi = eta2(3);
dphi = deta2(1);
dtheta = deta2(2);
dpsi = deta2(3);

dR_rpy = delRz(psi)*Ry(theta)*Rx(phi)*dpsi + Rz(psi)*delRy(theta)*Rx(phi)*dtheta + Rz(psi)*Ry(theta)*delRx(phi)*dphi;
end