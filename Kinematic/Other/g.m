function g_matrix = g( eta )
% This function computes the homogeneous transformation btw the inertial
% and some arbitrary frame.

p0 = eta(1:3);
R0 = R_rpy(eta(4:end));
g_matrix = [ R0, p0 ; 0 0 0 1 ];

end