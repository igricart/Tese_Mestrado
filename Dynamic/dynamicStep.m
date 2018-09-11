function Wrenches = dynamicStep( r1, r2, r3, V, dV, joint_type, q, Mass, Inertia, R, L, h, g_n_ef, BodyContact, BodyContactWrenches, BodyContactPositions )
% This function computes the backward propagation of forces and torques of
% the rigid body.

% Inputs:
% 'r1', 'r2' and 'r3' are the matrices representing the first, second and
% third columns (respectively) of the relative rotation matrices;
% 'V' and 'dV' are the matrices of velocities and accelerations of each
% bar frame. The last column is the camera frame vel./accel.;
% 'Mass' is a vector with the body masses;
% 'Inertia' is a matrix containing the inertia components of each body, wrt
% its center of mass;
% 'R' is the matrix of C.M. positions for each body wrt the local frame;
% 'L' is the matrix of joint displacement;
% 'h' is the matrix of axes vectors;
% 'g_n_ef' is the last link to end-effector hom. transf. matrix;
% 'BodyContact' is the vector of bodies where the wrenches are applied
% 'BodyContactWrenches' is the matrix of applied LOCAL wrenches
% 'BodyContactPositions' is the matrix of positions where the local force
% is applied.

% Outputs:
% 'F' is the matrix of joint wrenches. The first column is the base wrench,
% and the last is always zero.

%% Constants
zeros33 = zeros(3);
eye33 = eye(3);

num_bodies = size(V,2) - 1;
num_joints = num_bodies - 1;

Wrenches = zeros(6,num_bodies+1);
L_aux = zeros(3,num_bodies);

r1_aux = zeros(3,num_bodies);
r2_aux = zeros(3,num_bodies);
r3_aux = zeros(3,num_bodies);

r1_aux(:,end) = g_n_ef(1:3,1);
r2_aux(:,end) = g_n_ef(1:3,2);
r3_aux(:,end) = g_n_ef(1:3,3);

r1_aux(:,1:(end-1)) = r1;
r2_aux(:,1:(end-1)) = r2;
r3_aux(:,1:(end-1)) = r3;
L_aux(:,1:(end-1)) = L;

Axis = zeros(3,num_bodies);
Prismatic = zeros(1,num_bodies);
for i = 1:num_joints
    if joint_type(i) == 1
        Axis(:,i) = h(:,i)/norm(h(:,i));
        Prismatic(i) = q(i);
    else
        Axis(:,i) = zeros(3,1);
        Prismatic(i) = 0;
    end
end

%% Backward step iteration
for i = num_bodies:-1:1
    
    %% External wrenches computation
    TotalExternalWrenches = zeros(6,1);
    for j = 1:length(BodyContact)
        if BodyContact(j) == i-1
            PhiTContact = [ eye33 , zeros33 ;...
                            hat( BodyContactPositions(:,j) ), eye33 ];
            TotalExternalWrenches = TotalExternalWrenches + PhiTContact*BodyContactWrenches(:,j);
        end
    end
    
    %% Body terms computation
    Rot = [ r1_aux(:,i), r2_aux(:,i), r3_aux(:,i) ];
    J_R = [ Rot     , zeros33 ;...
            zeros33 , Rot     ];
    PhiT = [ eye33                             , zeros33 ;...
             hat( L_aux(:,i) + Prismatic(i)*Axis(:,i) ), eye33   ];
    Ici = [ Inertia(1,i), Inertia(4,i), Inertia(5,i) ;...
            Inertia(4,i), Inertia(2,i), Inertia(6,i) ;...
            Inertia(5,i), Inertia(6,i), Inertia(3,i) ];
    hat_R = hat(R(:,i));
    hat_omega = hat(V(4:end,i));
    hat_omega_v = hat_omega*V(1:3,i);
    off_term = Mass(i)*hat_R;
    Ii = Ici - off_term*hat_R;
    Mi = [ Mass(i)*eye33, -off_term ;...
           off_term      , Ii        ];
%     Bi = [ Mass(i)*(hat_omega^2*R(:,i)) ; hat_omega*(Ii*V(4:end,i)) ];
    Bi = [ Mass(i)*( (hat_omega^2)*R(:,i) + hat_omega_v ) ; hat_omega*(Ii*V(4:end,i)) + Mass(i)*hat_R*hat_omega_v ];
    
    Wrenches(:,i) = PhiT*J_R*Wrenches(:,i+1) + Mi*dV(:,i) + Bi - TotalExternalWrenches;
end

end

