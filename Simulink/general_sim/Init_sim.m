clear all; clc

global simulation_step;
global step_period;
simulation_step = 1e-4;

%% Number of joints (rad)
n = 4;

%% Masses (Kg)
mass_base = 0;
%mass_hip = 40.59;
mass_hip = 21.29;
mass_thigh = 8.57;
mass_shin = 2.29;
mass_foot = 1;

%% Inertia parameters (Kg-m^2)
I_base = [0 0 0 0 0 0]';
I_hip = [0 0 0 0 0 0]'; % (zeros because the hip does not rotates)
I_thigh = [0 0.435 0 0 0 0]';
I_shin = [0 0.062 0 0 0 0]';
I_foot = [0 0.018 0 0 0 0]';

%% Joint frame positions (from robot geometry)
l_base_hip = zeros(3,1);
l_hip_thigh = zeros(3,1);
l_thigh_shin = [ 0 0 1 ]';
l_shin_foot = [ 0 0 1 ]';

%% C.M. positions w.r.t. local frames (m)
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

%% PD
% %
% wn = 2*pi*8;
% zeta = 0.9;
% kd = 2*zeta*wn;
% ki = 0;
% kp = wn^2;
% Kp = kp*eye(n); Kd = kd*eye(n); Ki = ki*eye(n);
%% PID

wn = 2*pi*8;
zeta = 0.9;
p = 2*wn;
ki = p*wn^2;
kp = 2*zeta*wn*p + wn^2;
kd = p + 2*zeta*wn;
Kp = kp*eye(n); Kd = kd*eye(n); Ki = ki*eye(n);

%% Contact forces/torques
BodyContact = [ 4 4 ]; % two wrenches at body 4 (foot)
contact_point_h = [  0.1, 0, 0.1 ]'; % point h w.r.t. foot frame
contact_point_t = [ -0.3, 0, 0.1 ]'; % point t w.r.t. foot frame
BodyContactPositions = [ contact_point_h, contact_point_t ];

%% Contact model
s_z = 2 ; % height (ground till robot base) (m)
k_b = 5000; % spring constant (N/m, old = 10)
beta = 10; % gain for horizontal force measure (unitless, old = 10)

%% Gait Data from Spreadsheet
% readGaitData(file,row in spreadsheet, step period, time sample)
[data, t] = readGaitData('GaitData.xlsx',14,1,simulation_step);
step_period = t(end);
% concatData(data, time, window size for signal conditioning, number of
% steps)
data_steps = concatData(data,t,300,15);

%% Initial state |---
%                |
qInit = [ 0.0216 0.5325 -0.0255 -0.0481]';
dqInit = [ 0 0 0 0 ]';

qInit_model = qInit;
dqInit_model = (data_steps(2,2:5)' - data_steps(1,2:5)')/(data_steps(2,1) - data_steps(1,1));
 
% dq1 = (data_steps(1,2) - data_steps(2,2))/(data_steps(1,1) - data_steps(2,1));
% dq2 = (data_steps(1,3) - data_steps(2,3))/(data_steps(1,1) - data_steps(2,1));
% dq3 = (data_steps(1,4) - data_steps(2,4))/(data_steps(1,1) - data_steps(2,1));
% dq4 = (data_steps(1,5) - data_steps(2,5))/(data_steps(1,1) - data_steps(2,1));
% dqInit = [dq1 dq2 dq3 dq4]'; 


%% Model reference
wn_ref = 8*5;
zeta_ref = 0.7;

%% Computed Torque parameter mismatch
%param_error = (1 + 0.1*(rand()-1/2));
param_error = 1.0406;
param_error = 1;
Mass_ctrl = Mass_ctrl.*param_error;
Inertia_ctrl = Inertia_ctrl.*param_error;
R_ctrl = R_ctrl.*param_error;
L_ctrl = L_ctrl.*param_error;
h_ctrl = h_ctrl.*param_error;

%% HGO
Init_HGO;

%% Window size Noise estimation
%maximum value -> k_noise = 5e-7;
k_noise = 1e-6;
%k_noise = 0;
window_size = [60 20 20 40];

%% Computes INITIAL force state vector
[ ~, Fstate0 ] = ground_model( qInit, BodyContactPositions, s_z, L, h, beta,  k_b, joint_type );

%% Save command
% save('protese_noise_PID_HGOFeedback','ki','kd','kp','u','Mu','q_ref','dq_ref','q','dq','q_hat','dq_hat','simulation_step','step_period','param_error','qInit','dqInit','q_hat_init','dq_hat_init','window_size','SNR_used')
