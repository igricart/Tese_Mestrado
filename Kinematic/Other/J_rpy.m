function J = J_rpy(eta2)
%J_rpy(eta2) is the velocity transformation matrix Jb(eta2),
%eta2=[phi;theta;psi] is the RPY angles

R = R_rpy(eta2);
T = T_rpy(eta2);

J = [R zeros(3); zeros(3) T];

end