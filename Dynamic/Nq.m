function Nq = Nq( s, RPY, joint_type, q, Mass, Inertia, R, L, h, G, g_s_INS )
%This function computes the robot gravity vector.

num_joints = length(q);
dq = zeros(num_joints,1);
ddq = zeros(num_joints,1);

V0 = zeros(6,1);
dV0 = zeros(6,1);
g_n_ef = eye(4);

[ Nq, ~ ] = inverseDynamics( s, RPY, joint_type, q, dq, ddq, V0, dV0, Mass, Inertia, R, L, h, G, g_s_INS, g_n_ef, [], [], [] );

end