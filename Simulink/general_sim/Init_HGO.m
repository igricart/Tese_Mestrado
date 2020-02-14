%% Inicializador do HGO

% Dynamics
Ar = [zeros(4) eye(4); zeros(4) zeros(4)];
Br = [zeros(4); eye(4)];
Cr = [eye(4) zeros(4)];
L_hgo = [2*eye(4);6*eye(4)];
kpn = eye(4);

% Gain
Mu_hgo = 4e-4;
Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];

% Initial conditions
q_hat_init = zeros(4,1);
dq_hat_init = zeros(4,1);