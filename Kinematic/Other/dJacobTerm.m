function dJb_dq = dJacobTerm( s, joint_type, q, dq, L, h, g_s_INS, g_n_ef )
% This function computes the term dJcam*dq, where dJcam is the derivative
% of the body geometric Jacobian (from the INS to the camera).

% Inputs:
% 's' is the body where the INS is located (s = 0 is the ship);
% 'q' and 'dq' are the vectors of joint angles/velocities;
% 'L' is the matrix of joint displacement;
% 'h' is the matrix of unit axis (wrt the local frame);
% 'g_INS' is the transformation btw the bar frame and the INS frame, on each
% link. For simplicity, g_INS must be = eye(3) if s = 0. In other words, the 
% bar frame on the ship coincides with the INS frame;
% 'g_CAM' is the transformation btw the last bar frame and the camera
% frame.

% Outputs:
% 'dJdq'

num_joints = length(q);

V0INS = zeros(6,1);
dV0INS = zeros(6,1);
ddq = zeros(num_joints,1);

dJb_dq = zeros(6,1);
for i = 1:num_joints
    [ ~, ~, ~, ~, dV ] = kinematicStep( s, joint_type, q, dq, ddq, V0INS, dV0INS, L, h, g_s_INS, g_n_ef );
    dJb_dq = dV(:,end);
end

end