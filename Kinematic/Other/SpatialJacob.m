function Js = SpatialJacob( s, joint_type, q, L, h, g_s_INS, g_n_ef )
% This function computes the body geometric Jacobian of the mechanism (from
% the INS to the camera).

% Inputs:
% 's' is the body where the INS is located (s = 0 is the ship);
% 'q' is the vector of joint variables;
% 'L' is the matrix of joint displacement;
% 'h' is the matrix of unit axis (wrt the local frame);
% 'g_INS' is the transformation btw the bar frame and the INS frame, on each
% link. For simplicity, g_INS must be = eye(3) if s = 0. In other words, the 
% bar frame on the ship coincides with the INS frame.

% Outputs:
% 'Js' is the body geometric Jacobian.

Jb = BodyJacob( s, joint_type, q, L, h, g_s_INS, g_n_ef );
g_INSEFF = gINS2EFF( joint_type, q, L, h, g_s_INS, g_n_ef, s );
Js = Ad(g_INSEFF)*Jb;

end