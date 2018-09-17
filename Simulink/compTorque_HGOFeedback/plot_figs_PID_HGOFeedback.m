%% Script to plot data from .mat

load('protese_PID_stateFeedback.mat');
t = u.Time;
row_max = size(u.Data,2);
param_number = 5;

%% Error declaration

q_error = abs(q_ref.Data - q_hat.Data);
dq_error = abs(dq_ref.Data - dq_hat.Data);
q_est_error = abs(q.Data - q_hat.Data);
dq_est_error = abs(dq.Data - dq_hat.Data);

%% Plot XY limits for Control Signal and Errors
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

% Estimation error (rad)
x_lim_q_hat_error  = [
    0 4; 
    0 4; 
    0 4; 
    0 4];

y_lim_q_hat_error  = [
    0 1.5e-4; 
    0 1.5e-4; 
    0 1.5e-4; 
    0 1.5e-4];

% Estimation error (rad/s)
x_lim_dq_hat_error = [
    0 4; 
    0 4; 
    0 4; 
    0 4];

y_lim_dq_hat_error = [
    0 1; 
    0 1; 
    0 1; 
    0 1];

X_lim = [x_lim_u, x_lim_q_error, x_lim_dq_error, x_lim_q_hat_error, x_lim_dq_hat_error];
Y_lim = [y_lim_u, y_lim_q_error, y_lim_dq_error, y_lim_q_hat_error, y_lim_dq_hat_error];

%% Plot XY limits for Joint Position and Velocities

%Joint and estimated Joint angle (rad)
x_lim_q  = [
    0 4; 
    0 4; 
    0 4; 
    0 4];

y_lim_q  = [
    -0.2 0.2; 
    -0.2 0.7; 
    -1.2 0.0; 
    -0.8 0.0];

% Joint and estimated Joint Velocity (rad/s)
x_lim_dq  = [
    0 4; 
    0 4; 
    0 4; 
    0 4];

y_lim_dq  = [
    -7 8; 
    -7 8; 
    -7 8; 
    -7 8];


X2_lim = [x_lim_q, x_lim_dq];
Y2_lim = [y_lim_q, y_lim_dq];

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

% plot q_til
subplot(param_number,row_max,row_max + i);
plot(t, q_error(:,i));
title(['Tracking error from joint angle' num2str(i) '(rad)']);
xlim(X_lim(i,3:4));
ylim(Y_lim(i,3:4));

% plot dq_til
subplot(param_number,row_max,2*row_max + i);
plot(t, dq_error(:,i));
title(['Tracking error from joint velocity' num2str(i) '(rad/s)']);
xlim(X_lim(i,5:6));
ylim(Y_lim(i,5:6));

% plot q_hat_til
subplot(param_number,row_max,3*row_max + i);
plot(t, q_est_error(:,i));
title(['Estimation error from joint angle' num2str(i) '(rad)']);
xlim(X_lim(i,7:8));
ylim(Y_lim(i,7:8));

% plot dq_hat_til
subplot(param_number,row_max,4*row_max + i);
plot(t, dq_est_error(:,i));
title(['Estimation error from joint velocity' num2str(i) '(rad/s)']);
xlim(X_lim(i,9:10));
ylim(Y_lim(i,9:10));

end

%% Plot q q_hat dq dq_hat
f2 = figure();
grid on;

for i=1:row_max
% plot q and q_hat
subplot(2,row_max,i);
plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
title(['Joint angle and Estimated' num2str(i) '(rad)']);
xlim(X2_lim(i,1:2));
ylim(Y2_lim(i,1:2));

% plot dq and dq_hat
subplot(2,row_max,row_max + i);
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
title(['Joint velocity and Estimated' num2str(i) '(rad/s)']);
xlim(X2_lim(i,3:4));
ylim(Y2_lim(i,3:4));

end