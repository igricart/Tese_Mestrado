function MqV = MqV( s, joint_type, q, Mass, Inertia, R, L, h, g_s_INS )
%This function computes the robot MqV matrix.

num_joints = length(q);
dq = zeros(num_joints,1);
ddq = zeros(num_joints,1);

RPY = zeros(3,1);
V0 = zeros(6,1);
G = zeros(3,1);
g_n_ef = eye(4);

MqV = zeros(num_joints,6);
for i = 1:6
    dV0 = zeros(6,1); dV0(i) = 1;
    [ MqV(:,i), ~ ] = inverseDynamics( s, RPY, joint_type, q, dq, ddq, V0, dV0, Mass, Inertia, R, L, h, G, g_s_INS, g_n_ef, [], [], [] );
end

end