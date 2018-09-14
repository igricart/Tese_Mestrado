%% Inicializador do HGO

Ar = [zeros(4) eye(4); zeros(4) zeros(4)];
Br = [zeros(4); eye(4)];
Cr = [eye(4) zeros(4)];

P = conv(conv(conv([1 5 6],[1 5 6]),[1 5 6]),[1 5 6]);

Lc1 = [P(2:end)]';
Lc2 = [P(2:end)]';
Lc3 = [P(2:end)]';
Lc4 = [P(2:end)]';
L1 = [Lc1 Lc2 Lc3 Lc4];

Mu = 1*(1e-3);

L_hgo = [2*eye(4);6*eye(4)];
%L_hgo = L1;


Hmu = [eye(4)/Mu zeros(4); zeros(4) eye(4)/Mu^2];

