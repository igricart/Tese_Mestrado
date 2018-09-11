%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Passos para o uso da interface gráfica %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

------> Se deseja obter uma animação da simulação previamento realizada:

1) Primeiro, execute a simulação localizada na pasta "Simulink". A variável q (Timeseries) será automaticamente carregada no workspace do Matlab. 
Antes, não se esqueça de rodar o "Init_4DOF.m" contendo os parâmetros da simulação.

2) Com o Timeseries q carregado no workspace, execute o script "plot_prosthetics.m" para gerar a animação.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

------> Se desejar obter apenas uma figura com um configuração específica de juntas do robô:

1) Carregue os parâmetros do robô no workspace executando o "Init_4DOF.m", na pasta "Simulink";

2) Crie o objeto da classe Prosthetics com os parâmetros definidos em "Init_4DOF.m":

	protese = Prosthetics( l_base_hip, l_hip_thigh, l_thigh_shin, l_shin_foot, contact_point_h, contact_point_t, s_z );

3) Execute o método de plot da classe com um VETOR COLUNA q com os valores das juntas do robô. Os outros argumentos já foram declarados em "Init_4DOF.m".

	protese.PlotProsthetics( q, L, h, contact_point_h, contact_point_t, joint_type );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% ENJOY %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%