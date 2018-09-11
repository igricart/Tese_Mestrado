function [ r1, r2, r3, V, dV ] = kinematicStep( s, joint_type, q, dq, ddq, V0_INS, dV0_INS, L, h, g_s_INS, g_n_ef )
% This function computes the propagation of kinematic variables of a
% general kinematic chain.

% Inputs:
% 's' is the body where the INS is located (s = 0 is the ship);
% 'q', 'dq' and 'ddq' are the vectors of joint angles, velocities and
% accelerations;
% 'V0_link', 'dV0_link' are the body velocity/acceleration twists as measured
% wrt the INS frame on body s;
% 'L' is the matrix of joint displacement;
% 'h' is the matrix of unit axis (wrt the local frame);
% 'g_s_INS' is the transformation btw the bar frame and the INS frame, on each
% link. For simplicity, g_INS must be = eye(3) if s = 0. In other words, the 
% bar frame on the ship coincides with the INS frame.
% 'g_n_ef' is the transformation btw the last bar frame and the camera
% frame.

% Outputs:
% 'r1', 'r2' and 'r3' are the matrices representing the first, second and
% third columns (respectively) of the RELATIVE rotation matrices;
% 'V' and 'dV' are the matrices of velocities and accelerations of each
% bar frame. The last column is the camera frame vel./accel.;

%% Constants
zeros33 = zeros(3);
zeros31 = zeros(3,1);
zeros32 = zeros(3,2);
eye33 = eye(3);

num_joints = length(q);
num_bodies = num_joints+1;

V = zeros(6,num_bodies+1);
dV = zeros(6,num_bodies+1);

%% Initialize velocities/accelerations of the bar frame
Ad_g_INS = Ad(g_s_INS);
V(:,s+1) = Ad_g_INS*V0_INS;
dV(:,s+1) = Ad_g_INS*dV0_INS;

r1 = zeros(3,num_joints);
r2 = zeros(3,num_joints);
r3 = zeros(3,num_joints);

%% Forward step
for i = (s+1):num_joints
    switch joint_type(i)
        case 0 %% rotation
            Joint_vec = [ 0 ; q(i) ];
            dJoint_vec = [ 0 ; dq(i) ];
            ddJoint_vec = [ 0 ; ddq(i) ];
        case 1 %% prismatic
            Joint_vec = [ q(i) ; 0 ];
            dJoint_vec = [ dq(i) ; 0 ];
            ddJoint_vec = [ ddq(i) ; 0 ];
        otherwise %% rotation
            Joint_vec = [ 0 ; q(i) ];
            dJoint_vec = [ 0 ; dq(i) ];
            ddJoint_vec = [ 0 ; ddq(i) ];
    end
    axis = h(:,i)/norm(h(:,i));
    Rot = angvec2r( Joint_vec(2), axis );
    r1(:,i) = Rot(:,1); r2(:,i) = Rot(:,2); r3(:,i) = Rot(:,3);
    J_R = [ Rot      , zeros33 ;...
            zeros33 , Rot      ];
    Phi = [ eye33   , -( hat(L(:,i)) + hat(axis)*Joint_vec(1) ) ;...
            zeros33 ,  eye33                                    ];
    H = [ axis    , zeros31 ;...
          zeros31 , axis     ];
    hat_omega = hat(V(4:end,i));
    A = [ hat_omega*axis , hat( V(1:3,i) + hat_omega*L(:,i) )*axis ;...
             zeros31    , hat_omega*axis                           ];
    B = [ hat(hat_omega*axis)*axis , zeros31 ;... 
                                zeros32      ];
    V(:,i+1) = J_R'*(Phi*V(:,i) + H*dJoint_vec);
    dV(:,i+1) = J_R'*(Phi*dV(:,i) + H*ddJoint_vec + A*dJoint_vec + B*Joint_vec);
end

%% Transforms velocities/accelerations of the last frame to the camera frame
invAd_g_0ef = invAd(g_n_ef);
V(:,end) = invAd_g_0ef*V(:,end-1);
dV(:,end) = invAd_g_0ef*dV(:,end-1);

%% Backward step
for i = s:-1:1
    switch joint_type(i)
        case 0 %% rotation
            Joint_vec = [ 0 ; q(i) ];
            dJoint_vec = [ 0 ; dq(i) ];
            ddJoint_vec = [ 0 ; ddq(i) ];
        case 1 %% prismatic
            Joint_vec = [ q(i) ; 0 ];
            dJoint_vec = [ dq(i) ; 0 ];
            ddJoint_vec = [ ddq(i) ; 0 ];
        otherwise %% rotation
            Joint_vec = [ 0 ; q(i) ];
            dJoint_vec = [ 0 ; dq(i) ];
            ddJoint_vec = [ 0 ; ddq(i) ];
    end
    axis = h(:,i)/norm(h(:,i));
    Rot = angvec2r( Joint_vec(2), axis );
    r1(:,i) = Rot(:,1); r2(:,i) = Rot(:,2); r3(:,i) = Rot(:,3);
    J_R = [ Rot     , zeros33 ;...
            zeros33 , Rot          ];
    invPhi = [ eye33   , hat(L(:,i)) + hat(axis)*Joint_vec(1) ;...
               zeros33 , eye33                                ];
    H = [ axis    , zeros31 ;...
          zeros31 , axis    ];
    V(:,i) = invPhi*(J_R*V(:,i+1) - H*dJoint_vec);
      
    hat_omega = hat(V(4:end,i));
    A = [ hat_omega*axis , hat( V(1:3,i) + hat_omega*L(:,i) )*axis ;...
              zeros31    , hat_omega*axis                           ];
    B = [ hat(hat_omega*axis)*axis , zeros31 ;... 
                                zeros32      ];
    dV(:,i) = invPhi*(J_R*dV(:,i+1) - H*ddJoint_vec - A*dJoint_vec - B*Joint_vec);
end

end