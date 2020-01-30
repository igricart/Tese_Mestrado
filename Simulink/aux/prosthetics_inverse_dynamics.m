function u = prosthetics_inverse_dynamics(  q, dq, ddq, joint_type, Mass, Inertia, R, L, h, G )
%% This is the function for the prothetics control.

RPY = zeros(3,1);
V0INS = zeros(6,1);
dV0INS = zeros(6,1);

[ u, ~ ] = inverseDynamics( 0, RPY, joint_type, q, dq, ddq, V0INS, dV0INS, Mass, Inertia, R, L, h, G, eye(4,4), eye(4,4), [], [], [] );

end