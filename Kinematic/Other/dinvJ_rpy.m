function dinvJ_rpy = dinvJ_rpy(eta2,deta2)
%dinvJ_rpy(eta2,deta2) computes the time derivative of the inverse of the velocity transformation matrix Jb(psi,theta,phi)

dR = dR_rpy(eta2,deta2);
dinvT = dinvT_rpy(eta2,deta2);

dinvJ_rpy = [dR' zeros(3); zeros(3) dinvT];

end