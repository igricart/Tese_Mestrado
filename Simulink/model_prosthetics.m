function ddq = model_prosthetics( tau_q, q, dq, joint_type, Mass, Inertia, R, L, h, G, friction_torques, BodyContact, BodyContactWrenches, BodyContactPositions )
%% This is the function for the prothetics simulation.

%% Base is fixed
eta0 = zeros(3,1);
V0 = zeros(6,1);
dV0 = zeros(6,1);

% Direct dynamics
ddq = directDynamics( 0, tau_q, eta0, joint_type, q, dq, V0, dV0, Mass, Inertia, R, L, h, G, eye(4,4), eye(4,4), friction_torques, BodyContact, BodyContactWrenches, BodyContactPositions );

end