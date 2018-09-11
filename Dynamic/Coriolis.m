function Cq_term = Coriolis( s, joint_type, q, dq, V0INS, Mass, Inertia, R, L, h, g_s_INS )
%This function computes the vector of Coriolis torques.

num_joints = length(q);
ddq = zeros(num_joints,1);

RPY = zeros(3,1);
dV0INS = zeros(6,1);
G = zeros(3,1);
g_n_ef = eye(4);

[ Cq_term, ~ ] = inverseDynamics( s, RPY, joint_type, q, dq, ddq, V0INS, dV0INS, Mass, Inertia, R, L, h, G, g_s_INS, g_n_ef, [], [], [] );

end