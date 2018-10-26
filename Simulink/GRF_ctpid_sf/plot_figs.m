%% Script to plot data from .mat

load('protese_PID_stateFeedback.mat');
t = u.Time;
row_max = size(u.Data,2);
param_number = 3;

%% Define plot XY limits

%Control Signal
x_lim_u  = [
    0 4; 
    0 4; 
    0 4; 
    0 4];

y_lim_u  = [
    -800 800; 
    -800 800; 
    -300 300; 
    -100 100];

% Tracking error (rad)
x_lim_q_error  = [
    0 4; 
    0 4; 
    0 4; 
    0 4];

y_lim_q_error  = [
    0 0.04; 
    0 0.06; 
    0 0.08; 
    0 0.1];

% Tracking error (rad/s)
x_lim_dq_error = [
    0 4; 
    0 4; 
    0 4; 
    0 4];

y_lim_dq_error = [
    0 1; 
    0 1; 
    0 1; 
    0 1];

X_lim = [x_lim_u, x_lim_q_error, x_lim_dq_error];
Y_lim = [y_lim_u, y_lim_q_error, y_lim_dq_error];

%% Plot loop
f1 = figure();
grid on;

for i=1:row_max
% plot u
subplot(param_number,row_max,i);
plot(t, u.Data(:,i));
title(['Control Signal from joint' num2str(i) '(N.m/s)']);
xlim(X_lim(i,1:2));
ylim(Y_lim(i,1:2));

% plot q
subplot(param_number,row_max,row_max + i);
plot(t, q_error.Data(:,i));
title(['Tracking error from joint angle' num2str(i) '(rad)']);
xlim(X_lim(i,3:4));
ylim(Y_lim(i,3:4));

% plot dq
subplot(param_number,row_max,2*row_max + i);
plot(t, dq_error.Data(:,i));
title(['Tracking error from joint velocity' num2str(i) '(rad/s)']);
xlim(X_lim(i,5:6));
ylim(Y_lim(i,5:6));

end