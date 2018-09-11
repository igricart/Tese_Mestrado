function Mq = Mq( s, joint_type, q, Mass, Inertia, R, L, h, g_s_INS )
% This function computes the robot Mq matrix.

% Inputs:
% 's' is the body where the INS is located (s = 0 is the ship);
% 'q' is the vector of joint angles;
% 'Mass' is a vector with the body masses;
% 'Inertia' is a matrix containing the inertia components of each body, wrt
% its center of mass;
% 'R' is the matrix of C.M. positions for each body wrt the local frame;
% 'L' is the matrix of joint displacement.
% 'h' is the matrix of unit axis (wrt the local frame);

% Output:
% 'Mq' is the mass matrix wrt to the joint variables.

num_joints = length(q);
dq = zeros(num_joints,1);

RPY = zeros(3,1);
V0 = zeros(6,1);
dV0 = zeros(6,1);
G = zeros(3,1);
g_n_ef = eye(4);

Mq = zeros(num_joints);
for i = 1:num_joints
    ddq = zeros(num_joints,1); ddq(i) = 1;
    [ Mq(:,i), ~ ] = inverseDynamics( s, RPY, joint_type, q, dq, ddq, V0, dV0, Mass, Inertia, R, L, h, G, g_s_INS, g_n_ef, [], [], [] );
end

end