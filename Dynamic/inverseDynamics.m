function [ tau_q, Wrenches ] = inverseDynamics( s, RPY, joint_type, q, dq, ddq, V0INS, dV0INS, Mass, Inertia, R, L, h, G, g_s_INS, g_n_ef, BodyContact, BodyContactWrenches, BodyContactPositions )
% This function computes de inverse dynamics of a general mechanism.

% Inputs:
% 's' is the body where the INS is located (s = 0 is the ship);
% 'RPY' is the vector of roll, pitch and yaw angles, as measured by the
% INS;
% 'q', 'dq' and 'ddq' are the vectors of joint angles, velocities and
% accelerations;
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

% Outputs:
% 'tauq' is the vector of joint torques;
% 'F0' is the body wrench of the ship (wrt C.M.).

num_joints = length(q);
R_0INS = R_rpy(RPY);
dV0INS_with_gravity = dV0INS - [ (R_0INS')*G ; zeros(3,1) ];

[ r1, r2, r3, V, dV ] = kinematicStep( s, joint_type, q, dq, ddq, V0INS, dV0INS_with_gravity, L, h, g_s_INS, g_n_ef );
Wrenches = dynamicStep( r1, r2, r3, V, dV, joint_type, q, Mass, Inertia, R, L, h, g_n_ef, BodyContact, BodyContactWrenches, BodyContactPositions );

tau_q = zeros(num_joints,1);

zeros33 = zeros(3,3);
zeros31 = zeros(3,1);
for i = 1:num_joints
    Rot = [ r1(:,i), r2(:,i), r3(:,i) ];
    J_R = [ Rot     , zeros33 ;...
            zeros33 , Rot     ];
    switch joint_type(i)
        case 0 %% rotational
            H = [ zeros31 ; h(:,i) ];
        case 1 %% prismatic
            H = [ h(:,i) ; zeros31 ];
        otherwise %% rotational
            H = [ zeros31 ; h(:,i) ];
    end
    tau_q(i) = (H')*J_R*Wrenches(:,i+1);
end

end