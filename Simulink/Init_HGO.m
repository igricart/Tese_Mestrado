%% Inicializador do HGO

% Dynamics
Ar = [zeros(4) eye(4); zeros(4) zeros(4)];
Br = [zeros(4); eye(4)];
Cr = [eye(4) zeros(4)];
L_hgo = [2*eye(4);6*eye(4)];
kpn = eye(4);

% Gain
Mu = 1*(1e-3);
Hmu = [eye(4)/Mu zeros(4); zeros(4) eye(4)/Mu^2];

% Initial conditions
q_hat_init = zeros(4,1);
% q_hat_init = qInit;
dq_hat_init = zeros(4,1);