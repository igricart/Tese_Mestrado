%% Script to plot data from .mat

load('protese_PID_stateFeedback.mat');
t = u.Time;
row_max = size(u.Data,2);
param_number = 5;
t_end = 1;
time_axis = [0 t_end];
%% Error declaration

q_error = abs(q_ref.Data - q_hat.Data);
dq_error = abs(dq_ref.Data - dq_hat.Data);
q_est_error = abs(q.Data - q_hat.Data);
dq_est_error = abs(dq.Data - dq_hat.Data);

%% Units declaration

unit = [
    "(N.m/s)";
    "(m)";
    "(m/s)";
    "(rad)";
    "(rad/s)";
    "(N)";
    ];

%% Plot XY limits for Control Signal and Errors
%Control Signal
y_lim_u  = [
    -800 800; 
    -800 800; 
    -300 300; 
    -100 100];

% Tracking error (m or rad)
y_lim_q_error  = [
    0 0.04; 
    0 0.06; 
    0 0.08; 
    0 0.1];

% Tracking error (m/s or rad/s)
y_lim_dq_error = [
    0 1; 
    0 1; 
    0 1; 
    0 1];

% Estimation error (m or rad)
y_lim_q_hat_error  = [
    0 1.5e-4; 
    0 1.5e-4; 
    0 1.5e-4; 
    0 1.5e-4];

% Estimation error (m/s or rad/s)
y_lim_dq_hat_error = [
    0 1; 
    0 1; 
    0 1; 
    0 1];

Y_lim = [y_lim_u, y_lim_q_error, y_lim_dq_error, y_lim_q_hat_error, y_lim_dq_hat_error];


%% General Plot loop
f1 = figure();
grid on;

% for i=1:row_max
% % plot u
% subplot(param_number,row_max,i);
% plot(t, u.Data(:,i));
% title(['Control Signal from joint' num2str(i) '(N.m/s)']);
% xlim(time_axis);
% ylim(Y_lim(i,1:2));
% 
% % plot q_til
% subplot(param_number,row_max,row_max + i);
% plot(t, q_error(:,i));
% if (i == 1)
%     title(['Tracking error from joint angle' num2str(i) '(m)']);
% else
%     title(['Tracking error from joint angle' num2str(i) '(rad)']);
% end
% xlim(time_axis);
% ylim(Y_lim(i,3:4));
% 
% % plot dq_til
% subplot(param_number,row_max,2*row_max + i);
% plot(t, dq_error(:,i));
% if (i == 1)
%     title(['Tracking error from joint velocity' num2str(i) '(m/s)']);
% else
%     title(['Tracking error from joint velocity' num2str(i) '(rad/s)']);
% end
% xlim(time_axis);
% ylim(Y_lim(i,5:6));
% 
% % plot q_hat_til
% subplot(param_number,row_max,3*row_max + i);
% plot(t, q_est_error(:,i));
% if (i == 1)
%     title(['Estimation error from joint angle' num2str(i) '(m)']);
% else
%     title(['Estimation error from joint angle' num2str(i) '(rad)']);
% end
% xlim(time_axis);
% ylim(Y_lim(i,7:8));
% 
% % plot dq_hat_til
% subplot(param_number,row_max,4*row_max + i);
% plot(t, dq_est_error(:,i));
% if (i == 1)
%     title(['Estimation error from joint velocity' num2str(i) '(m/s)']);
% else
%     title(['Estimation error from joint velocity' num2str(i) '(rad/s)']);
% end
% xlim(time_axis);
% ylim(Y_lim(i,9:10));
% 
% end

%% Plot XY limits for Joint Position and Velocities
%Joint and estimated Joint angle
y_lim_q  = [
    -0.2 0.2; 
    -0.2 0.7; 
    -1.2 0.0; 
    -0.8 0.0];

% Joint and estimated Joint Velocity
y_lim_dq  = [
    -10 10; 
    -10 10; 
    -10 10; 
    -10 10];

Y2_lim = [y_lim_q, y_lim_dq];

%% Plot q q_hat dq dq_hat
f2 = figure();
grid on;

for i=1:row_max
% plot q and q_hat
subplot(2,row_max,i);
plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
if (i == 1)
    title(['Joint angle and Estimated' num2str(i) '(m)']);
else
   title(['Joint angle and Estimated' num2str(i) '(rad)']);
end
xlim(time_axis);
ylim(Y2_lim(i,1:2));

% plot dq and dq_hat
subplot(2,row_max,row_max + i);
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
if (i == 1)
    title(['Joint velocity and Estimated' num2str(i) '(m/s)']);
else
   title(['Joint velocity and Estimated' num2str(i) '(rad/s)']);
end
xlim(time_axis);
ylim(Y2_lim(i,3:4));

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot 1 Article
% Joint and estimated joint angle on the left subplot and a zoomed plot on the
% right

% Joints zoomed
x_lim_q_zoom  = [
    0 0.006; 
    0 0.01; 
    0 0.01; 
    0 0.01];

y_lim_q_zoom  = [
    -0.02 0.04; 
    -0.0 1.0; 
    -0.19 0.0; 
    -0.6 0.0];

X_lim_q_zoom = x_lim_q_zoom;
Y_lim_q_zoom = y_lim_q_zoom;

% Plot 1 - Hip Displacement
i = 1;
f_hip_q = figure();

subplot(1,2,1);
plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
ylabel(['Hip displacement (m)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,1:2));

% Zoomed
subplot(1,2,2);
plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
ylabel(['Hip displacement (m)']);
xlabel(['Time (sec)']);
xlim(X_lim_q_zoom(i,1:2));
ylim(Y_lim_q_zoom(i,1:2));
legend({'True','Estimated'},'Location','southwest');
saveas(f_hip_q,['q_hip_mu_' num2str(sprintf('%.0d',Mu))],'epsc');

% Plot 2 - Thigh angle
% Thigh joint zoomed
i = 2;
f_thigh_q = figure();

subplot(1,2,1);
plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
ylabel(['Thigh angle (rad)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,1:2));

% Zoomed
subplot(1,2,2);
plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
ylabel(['Thigh angle (rad)']);
xlabel(['Time (sec)']);
xlim(X_lim_q_zoom(i,1:2));
ylim(Y_lim_q_zoom(i,1:2));
legend({'True','Estimated'},'Location','southwest');
saveas(f_thigh_q,['q_thigh_mu_' num2str(sprintf('%.0d',Mu))],'epsc');

% Plot 3 - Knee angle
% Knee joint zoomed
i = 3;
f_knee_q = figure();

subplot(1,2,1);
plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
ylabel(['Knee angle (rad)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,1:2));

% Zoomed
subplot(1,2,2);
plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
ylabel(['Knee angle (rad)']);
xlabel(['Time (sec)']);
xlim(X_lim_q_zoom(i,1:2));
ylim(Y_lim_q_zoom(i,1:2));
legend({'True','Estimated'},'Location','southwest');
saveas(f_knee_q,['q_knee_mu_' num2str(sprintf('%.0d',Mu))],'epsc');

% Plot 4 - Ankle angle
% Ankle joint zoomed
i = 4;
f_ankle_q = figure();

subplot(1,2,1);
plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
ylabel(['Ankle angle (rad)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,1:2));

% Zoomed
subplot(1,2,2);
plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
ylabel(['Ankle angle (rad)']);
xlabel(['Time (sec)']);
xlim(X_lim_q_zoom(i,1:2));
ylim(Y_lim_q_zoom(i,1:2));
legend({'True','Estimated'},'Location','southwest');
saveas(f_ankle_q,['q_ankle_mu_' num2str(sprintf('%.0d',Mu))],'epsc');

%% Plot 2 Article
% Velocity value and estimation

% Hip joint zoomed
x_lim_q_zoom  = [
    0 0.006; 
    0 0.01; 
    0 0.01; 
    0 0.01];

y_lim_q_zoom  = [
    -0.02 0.04; 
    -0.0 1.0; 
    -0.19 0.0; 
    -0.6 0.0];

X_lim_q_zoom = x_lim_q_zoom;
Y_lim_q_zoom = y_lim_q_zoom;

% Plot Velocitys
f_joints_dq = figure();

subplot(2,2,1);
i = 1;
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Hip velocity (m/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));
legend({'True','Estimated'},'Location','southwest');

subplot(2,2,2);
i = 2;
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Thigh angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));
legend({'True','Estimated'},'Location','southwest');

subplot(2,2,3);
i = 3;
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Knee angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));
legend({'True','Estimated'},'Location','southwest');

subplot(2,2,4);
i = 4;
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Ankle angular velocity (rad)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));
legend({'True','Estimated'},'Location','southwest');

saveas(f_joints_dq,['joint_velocities_mu_' num2str(sprintf('%.0d',Mu))],'epsc');

%% Plot 3 Article
% Joint and estimated joint velocities on the left subplot and a zoomed plot on the
% right

% Joints zoomed
x_lim_dq_zoom  = [
    0 0.05; 
    0 0.02; 
    0 0.015; 
    0 0.02];

y_lim_dq_zoom  = [
    -10  20; 
    -20  20; 
    -20  20; 
    -20  40];

X_lim_dq_zoom = x_lim_dq_zoom;
Y_lim_dq_zoom = y_lim_dq_zoom;

% Plot 1 - Hip Velocity
i = 1;
f_hip_dq = figure();

subplot(1,2,1);
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Hip velocity (m/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));

% Zoomed
subplot(1,2,2);
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Hip velocity (m/s)']);
xlabel(['Time (sec)']);
xlim(X_lim_dq_zoom(i,1:2));
ylim(Y_lim_dq_zoom(i,1:2));
legend({'True','Estimated'},'Location','southwest');
saveas(f_hip_dq,['dq_hip_mu_' num2str(sprintf('%.0d',Mu))],'epsc');

% Plot 2 - Thigh angular velocity
% Thigh joint zoomed
i = 2;
f_thigh_dq = figure();

subplot(1,2,1);
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Thigh angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));

% Zoomed
subplot(1,2,2);
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Thigh angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(X_lim_dq_zoom(i,1:2));
ylim(Y_lim_dq_zoom(i,1:2));
legend({'True','Estimated'},'Location','southwest');
saveas(f_thigh_dq,['dq_thigh_mu_' num2str(sprintf('%.0d',Mu))],'epsc');

% Plot 3 - Knee angular velocity
% Knee joint zoomed
i = 3;
f_knee_dq = figure();

subplot(1,2,1);
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Knee angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));

% Zoomed
subplot(1,2,2);
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Knee angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(X_lim_dq_zoom(i,1:2));
ylim(Y_lim_dq_zoom(i,1:2));
legend({'True','Estimated'},'Location','southwest');
saveas(f_knee_dq,['dq_knee_mu_' num2str(sprintf('%.0d',Mu))],'epsc');

% Plot 4 - Ankle angle
% Ankle joint zoomed
i = 4;
f_ankle_dq = figure();

subplot(1,2,1);
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Ankle angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));

% Zoomed
subplot(1,2,2);
plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Ankle angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(X_lim_dq_zoom(i,1:2));
ylim(Y_lim_dq_zoom(i,1:2));
legend({'True','Estimated'},'Location','southwest');
saveas(f_ankle_dq,['dq_ankle_mu_' num2str(sprintf('%.0d',Mu))],'epsc');

