%% Init control signals to denoise
clear all; clc;

%% Empirical SNR
global simulation_step;
global bufferNSR;
global window_size;
global step_period;
simulation_step = 1e-3;
step_period = 1;

% Empirical SNR estimation parameters 
window_size = [10 10 10 10];
bufferNSR=zeros(0.05/simulation_step,1);

%% Load control signal data
controlSignal = open('u_stateFeedback.mat');
power_ref = calcPower(controlSignal.u.Data);

%% Low pass filter parameters
filter_pole = 30;

%% Run simulation
%sym('low_pass_filter.slx');

%% Output
%power_u_filtered = calcPower(u_filtered.Data);
