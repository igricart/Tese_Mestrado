clear all; clc

%% Number of joints
n = 4;

%% Masses
mass_base = 0;
mass_hip = 21.29;
mass_thigh = 8.57;
mass_shin = 2.33;
mass_foot = 1;

%% Inertia parameters
I_base = [0 0 0 0 0 0]';
I_hip = [0 0 0 0 0 0]'; % (zeros because the hip does not rotates)
I_thigh = [0 0.435 0 0 0 0]';
I_shin = [0 0.062 0 0 0 0]';
I_foot = [0 0.062 0 0 0 0]';

%% Joint frame positions (from robot geometry)
l_base_hip = zeros(3,1);
l_hip_thigh = zeros(3,1);
l_thigh_shin = [ 0 0 1 ]';
l_shin_foot = [ 0 0 1 ]';

%% C.M. positions w.r.t. local frames
r_base = zeros(3,1);
r_hip = [ 0 0 0 ]';
r_thigh = [ 0 0 0.5 ]';
r_shin = [ 0 0 0.5 ]';
r_foot = [ 0 0 0.1 ]';

%% Gravity acceleration
g = 9.81;
ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

%% Simulation parameters
joint_type = [ 1 0 0 0 ];
Mass = [ mass_base mass_hip mass_thigh mass_shin mass_foot ];
Inertia = [ I_base I_hip I_thigh I_shin I_foot ];
R = [ r_base r_hip r_thigh r_shin r_foot ]; % C.M. positions
L = [ l_base_hip l_hip_thigh l_thigh_shin l_shin_foot ]; % joint positions
h = [ ez -ey -ey -ey ]; % axes
G = g*ez; % gravity

%% Control parameters
Mass_ctrl = [ mass_base mass_hip mass_thigh mass_shin mass_foot ];
Inertia_ctrl = [ I_base I_hip I_thigh I_shin I_foot ];
R_ctrl = [ r_base r_hip r_thigh r_shin r_foot ];
L_ctrl = [ l_base_hip l_hip_thigh l_thigh_shin l_shin_foot ];
h_ctrl = [ ez -ey -ey -ey ];
G_ctrl = g*ez;

%% Initial state |---
%                |
qInit = [ 0 pi/2 -pi/2 0 ]';
dqInit = [ 0 0 0 0 ]';

%% CTPID Gains
kp = 20; kd = 10; ki = 0; 
Kp = kp*eye(n); Kd = kd*eye(n); Ki = ki*eye(n);

%% Contact forces/torques
BodyContact = [ 4 4 ]; % two wrenches at body 4 (foot)
contact_point_h = [  0.1, 0, 0.1 ]'; % point h w.r.t. foot frame
contact_point_t = [ -0.1, 0, 0.1 ]'; % point t w.r.t. foot frame
BodyContactPositions = [ contact_point_h, contact_point_t ];

%% Contact model
s_z = 2 ; % height (ground till robot base)
k_b = 10; % spring constant
beta = 10; % spring constant

%% Joint reference values
Amp = [ 1 ; pi/2*ones(3,1) ];
w = [ 0   ; 0.5*pi*ones(3,1) ];
offset = zeros(n,1);

%% Computes INITIAL force state vector
[ ~, Fstate0 ] = ground_model( qInit, BodyContactPositions, s_z, L, h, beta,  k_b, joint_type );
