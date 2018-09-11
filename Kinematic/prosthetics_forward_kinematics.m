function [ P_point, R_body ] = prosthetics_forward_kinematics( q, L, h, body, point, joint_type )
%% This function computes the foot forward kinematics
% 'q' - joint vector;
% 'L' - matrix of joint displacements;
% 'h' - matrix of joint axes
% 'body' - number of body where the point is (0 is the base body)
% 'point' - point on local frame of the respective body
% 'joint_type' - vector of joint types (0 - rotational, 1- prismatic)

% 'P_point' - the inertial 3D location of the point
% 'R_body' - rotation matrix of body wrt inertial

[ r1, r2, r3, P ] = poseKinematics( 0, joint_type, zeros(6,1), q, L, h, eye(4), eye(4) );
R_body = [ r1(:,body+1), r2(:,body+1), r3(:,body+1) ];

P_point = P(:,body+1) + R_body*point;

end