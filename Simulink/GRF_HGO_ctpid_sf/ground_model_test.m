function [ GroundWrenches, ForceState ] = ground_model_test( q, BodyContactPositions, s_z, L, h, beta,  k_b, joint_type )
% function BodyContactWrenches = fcn( t, q, L, h, BodyContactPositions, joint_type, sz, beta, kb)
% Esta fun��o calcula as for�as de contato com o ch�o

%% Transforma as coordenadas locais dos pontos de contato em coordenadas inerciais
body = 4; % foot
contact_point_h = BodyContactPositions(:,1);
contact_point_t = BodyContactPositions(:,2);
[ P_h, R_h ] = prosthetics_forward_kinematics( q, L, h, body, contact_point_h, joint_type ); % posi��o do ponto h
[ P_t, R_t ] = prosthetics_forward_kinematics( q, L, h, body, contact_point_t, joint_type ); % posi��o do ponto t

%% Modelo de contato (em coordenadas inerciais)
z_h = cos(q);
z_t = cos(q);

F_zh = -(k_b/2)*cos(q);%*(1+sign(z_h - s_z));
F_zt = -(k_b/2)*cos(q);%*(1+sign(z_t - s_z));

F_xh = F_zh;
F_xt = F_zt;

ForceState = [ F_zh F_zt F_xh F_xt ]'; 

Fh_ground = [ F_xh 0 F_zh ]';
Ft_ground = [ F_xt 0 F_zt ]';

%% Transforma para coord. locais
Fh_ground_local = (R_h')*Fh_ground;
Ft_ground_local = (R_t')*Ft_ground;
GroundWrenches = [ Fh_ground_local, Ft_ground_local ; zeros(3,2) ];