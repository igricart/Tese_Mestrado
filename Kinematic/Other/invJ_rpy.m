function invJ = invJ_rpy(eta2)
%invJ_rpy(eta2) is the inverse of the velocity transformation matrix Jb(eta2),
%eta2=[phi;theta;psi] is the RPY angles

R = R_rpy(eta2);
invT = invT_rpy(eta2);

invJ = [R' zeros(3); zeros(3) invT];

end