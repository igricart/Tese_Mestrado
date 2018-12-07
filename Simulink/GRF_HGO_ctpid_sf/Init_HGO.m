%% Inicializador do HGO

% Dynamics
Ar = [zeros(4) eye(4) zeros(4); zeros(8,12)];
Br = [zeros(4); eye(4); zeros(4)];
Cr = [eye(4) zeros(4) zeros(4)];
L_hgo = [10*eye(4);10*eye(4);10*eye(4)];
kpn = eye(4);

% Gain
Mu = 1*(1e-3);
Hmu = [eye(4)/Mu zeros(4) zeros(4); zeros(4) eye(4)/Mu^2 zeros(4); zeros(4) zeros(4) eye(4)/Mu^3];

% Initial conditions
q_hat_init = zeros(4,1);
q_hat_init = [ 0.0216 0.5675 -0.13 -0.39 ]';
% q_hat_init = qInit;
dq_hat_init = zeros(4,1);
gfr_init = zeros(4,1);
