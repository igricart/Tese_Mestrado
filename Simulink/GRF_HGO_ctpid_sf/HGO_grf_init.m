%% Inicializador do HGO de Ground Force Reaction
%n = number of "states"
n = 4;
% Dynamics
Ar_mod = [zeros(n) eye(n); zeros(n) zeros(n)];
Br_mod = [zeros(n); eye(n)];
Cr_mod = [eye(n) zeros(n); zeros(n) eye(n)];
L_hgo_mod = [5*eye(n);10*eye(n)];
kpn_mod = eye(n);

% Gain
Mu_mod = 1*(1e-4);
Hmu_mod = [eye(n)/Mu_mod zeros(n); zeros(n) eye(n)/Mu_mod^2 ];

% Initial conditions
grf_init = 0; 