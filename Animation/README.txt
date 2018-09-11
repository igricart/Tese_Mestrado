%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Passos para o uso da interface gr�fica %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

------> Se deseja obter uma anima��o da simula��o previamento realizada:

1) Primeiro, execute a simula��o localizada na pasta "Simulink". A vari�vel q (Timeseries) ser� automaticamente carregada no workspace do Matlab. 
Antes, n�o se esque�a de rodar o "Init_4DOF.m" contendo os par�metros da simula��o.

2) Com o Timeseries q carregado no workspace, execute o script "plot_prosthetics.m" para gerar a anima��o.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

------> Se desejar obter apenas uma figura com um configura��o espec�fica de juntas do rob�:

1) Carregue os par�metros do rob� no workspace executando o "Init_4DOF.m", na pasta "Simulink";

2) Crie o objeto da classe Prosthetics com os par�metros definidos em "Init_4DOF.m":

	protese = Prosthetics( l_base_hip, l_hip_thigh, l_thigh_shin, l_shin_foot, contact_point_h, contact_point_t, s_z );

3) Execute o m�todo de plot da classe com um VETOR COLUNA q com os valores das juntas do rob�. Os outros argumentos j� foram declarados em "Init_4DOF.m".

	protese.PlotProsthetics( q, L, h, contact_point_h, contact_point_t, joint_type );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% ENJOY %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%