function [ddq, tau_gfr] = directDynamics( s, tauq, RPY, joint_type, q, dq, V0INS, dV0INS, Mass, Inertia, R, L, h, G, g_s_INS, g_n_ef, friction_torques, BodyContact, BodyContactWrenches, BodyContactPositions )
% This function computes de direct dynamics of a general mechanism.

% Inputs:
% 's' is the body where the INS is located (s = 0 is the ship);
% 'tauq' is the vector of joint torques;
% 'RPY' is the vector of roll, pitch and yaw angles, as measured by the
% INS;
% 'q' and 'dq' are the vectors of joint angles and velocities;
% 'V0INS', 'dV0INS' are the body velocity/acceleration twists as measured
% wrt the INS frame;
% 'Mass' is a vector with the body masses;
% 'Inertia' is a matrix containing the inertia components of each body, wrt
% its center of mass;
% 'R' is the matrix of C.M. positions for each body wrt the local frame;
% 'L' is the matrix of joint displacement;
% 'h' is the matrix of unit axis (wrt the local frame);
% 'G' is the gravity vector, wrt to the inertial frame;
% 'g_INS' is the transformation btw the bar frame and the INS frame, on each
% link. For simplicity, g_INS must be = eye(3) if s = 0. In other words, the 
% bar frame on the ship coincides with the INS frame.
% 'g_CAM' is the transformation btw the last bar frame and the camera
% frame.
% 'friction_torques' are exactly what they mean.

% Outputs:
% 'ddq' is the vector of joint accelerations;

num_joints = length(q);
ddq = zeros(num_joints,1);
zeroContactWrenches = zeros (6,2);
[ tauq_line, ~ ] = inverseDynamics( s, RPY, joint_type, q, dq, ddq, V0INS, dV0INS, Mass, Inertia, R, L, h, G, g_s_INS, g_n_ef, BodyContact, BodyContactWrenches, BodyContactPositions );
[ tauq_noContact, ~ ] = inverseDynamics( s, RPY, joint_type, q, dq, ddq, V0INS, dV0INS, Mass, Inertia, R, L, h, G, g_s_INS, g_n_ef, BodyContact, zeroContactWrenches, BodyContactPositions );
ddq = Mq( s, joint_type, q, Mass, Inertia, R, L, h, g_s_INS )\( tauq - tauq_line - friction_torques );
tau_gfr = tauq_line - tauq_noContact;
end