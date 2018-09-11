%% Inicializador do HGO

Ar = [zeros(4) eye(4); zeros(4) zeros(4)];
Br = [zeros(4); eye(4)];
Cr = [eye(4) zeros(4)];

P = conv(conv(conv([1 2 1],[1 2 1]),[1 2 1]),[1 2 1]);

Lc1 = [P(2:end)]';
Lc2 = [P(2:end)]';
Lc3 = [P(2:end)]';
Lc4 = [P(2:end)]';
L1 = [Lc1 Lc2 Lc3 Lc4];

Mu = 5e-3;

L_hgo = [eye(4);2*eye(4)];
%L_hgo = L1;


Hmu = [eye(4)/Mu zeros(4); zeros(4) eye(4)/Mu^2];

kpn = eye(4);

passo = 1e-4;

buffer2=zeros(0.0005/passo,1);
buffer3=zeros(0.0005/passo,1);

buffersup=zeros(0.0005/passo,1);
bufferinf=zeros(0.0005/passo,1);
bufferNSR=zeros(0.05/passo,1);

% 
% e1=0.8;e2=0.1;
% y1=0.1/8;y2=0.1;
% aux=inv([e1 1;e2 1])*[y1 y2]';
% mua=aux(1);
% mub=aux(2);


