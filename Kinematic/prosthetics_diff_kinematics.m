function V_point = prosthetics_diff_kinematics( q, dq, L, h, body, point, joint_type )
% This function computes the velocity twist of the given point at one of 
% the bodies of the kinematic chain.

zeros61 = zeros(6,1);
n = length(q);
ddq = zeros(n,1);

[ ~, ~, ~, V, ~ ] = kinematicStep( 0, joint_type, q, dq, ddq, zeros61, zeros61, L, h, eye(4), eye(4) );

point_transform = [ eye(3), point ;...
                    0 0 0 ,   1   ];
V_point = Ad(point_transform)*V(:,body+1);

end