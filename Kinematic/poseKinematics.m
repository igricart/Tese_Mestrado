function [ r1, r2, r3, P ] = poseKinematics( s, joint_type, eta_INS, q, L, h, g_INS, g_n_ef )
% This function computes the forward kinematics of a general mechanism.

% Inputs:
% 's' is the body where the INS is located (s = 0 is the ship);
% 'eta_INS' is the position + RPY angles measured by the sensor;
% 'q' is the vector of joint variables;
% 'L' is the matrix of joint displacement;
% 'h' is the matrix of unit axis (wrt the local frame);
% 'g_INS' is the transformation btw the bar frame and the INS frame, on each
% link. For simplicity, g_INS must be = eye(3) if s = 0. In other words, the 
% bar frame on the ship coincides with the INS frame.
% 'g_CAM' is the transformation btw the last bar frame and the camera focal
% plane.

% Outputs:
% 'r1', 'r2' and 'r3' are the matrices representing the first, second and
% third columns (respectively) of the ABSOLUTE link rotation matrices;
% 'P' is the matrix of inertial positions of each body frame. The last
% column is the end-effector frame position.

num_joints = length(q);
num_bodies = num_joints + 1;

zeros3D = zeros(3,num_bodies+1);
r1 = zeros3D;
r2 = zeros3D;
r3 = zeros3D;
P = zeros3D;

%% INS variables
eta1_INS = eta_INS(1:3);
eta2_INS = eta_INS(4:end);

%% Converting to local frame (joint frame, in the case of s~=0, and bar frame, if s=0)
Ri = R_rpy(eta2_INS);
R0 = Ri*( g_INS(1:3,1:3) ).';
p0 = eta1_INS - R0*g_INS(1:3,4);
% p0 = eta1_INS - Ri*g_INS(1:3,4);

%% Initialize positions and rotation matrices
P(:,s+1) = p0;
r1(:,s+1) = R0(:,1);
r2(:,s+1) = R0(:,2);
r3(:,s+1) = R0(:,3);

%% Forward step
for i = (s+1):num_joints
    axis = h(:,i)/norm(h(:,i));
    switch joint_type(i)
        case 0 %% rotation
            Joint_vec = [ 0 ; q(i) ];
            R_rel = angvec2r( Joint_vec(2), axis );
        case 1 %% prismatic
            Joint_vec = [ q(i) ; 0 ];
            R_rel = eye(3,3);
        otherwise %% rotation
            Joint_vec = [ 0 ; q(i) ];
            R_rel = angvec2r( Joint_vec(2), axis );
    end
    Ri = [ r1(:,i), r2(:,i), r3(:,i) ];
    
    P(:,i+1) = P(:,i) + Ri*(L(:,i) + axis*Joint_vec(1));
    R_next = Ri*R_rel;
    
    r1(:,i+1) = R_next(:,1);
    r2(:,i+1) = R_next(:,2);
    r3(:,i+1) = R_next(:,3);
end

%% Transforms from last bar frame to camera frame
R_next = [ r1(:,end-1), r2(:,end-1), r3(:,end-1) ];
P(:,end) = P(:,end-1) + R_next*g_n_ef(1:3,4);
R_end = R_next*g_n_ef(1:3,1:3);
r1(:,end) = R_end(:,1);
r2(:,end) = R_end(:,2);
r3(:,end) = R_end(:,3);

%% Backward step
for i = s:-1:1
    axis = h(:,i)/norm(h(:,i));
    switch joint_type(i)
        case 0 %% rotation
            Joint_vec = [ 0 ; q(i) ];
            R_rel = angvec2r( Joint_vec(2), axis );
        case 1 %% prismatic
            Joint_vec = [ q(i) ; 0 ];
            R_rel = eye(3,3);
        otherwise %% rotation
            Joint_vec = [ 0 ; q(i) ];
            R_rel = angvec2r( Joint_vec(2), axis );
    end
    
    Ri = [ r1(:,i+1), r2(:,i+1), r3(:,i+1) ];
    R_previous = Ri*(R_rel.');
    P(:,i) = P(:,i+1) - R_previous*(L(:,i) + axis*Joint_vec(1));
    
    r1(:,i) = R_previous(:,1);
    r2(:,i) = R_previous(:,2);
    r3(:,i) = R_previous(:,3);
end

end