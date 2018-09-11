function dJ = dJ_rpy(eta2,deta2)
%dJ_rpy(eta2,deta2) computes the time derivative of the velocity transformation matrix Jb(eta2)

dR = dR_rpy(eta2,deta2);
dT = dT_rpy(eta2,deta2);

dJ = [dR zeros(3); zeros(3) dT];

end