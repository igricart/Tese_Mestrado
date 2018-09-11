function g_0eff = g0eff( eta_INS, joint_type, q, L, h, g_s_INS, g_n_ef, s )
% This function computes the homogeneous transformation btw the inertial
% and the camera frame.

[ r1, r2, r3, P ] = poseKinematics( s, joint_type, eta_INS, q, L, h, g_s_INS, g_n_ef );
g_0eff = [ r1(:,end), r2(:,end), r3(:,end), P(:,end) ; 0 0 0 1 ];

end