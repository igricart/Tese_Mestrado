%% Script to plot data from .mat
% Simulation with Computed Torque, PID, HGO Observer and noise added

% load('protese_noise_PID_var_HGOFeedback.mat');
% load('protese_noise_PID_19e-4_HGOFeedback.mat');
% load('protese_noise_PID_4e-4_HGOFeedback.mat');
load('protese_noise_PID_10e-4_HGOFeedback.mat');

t = u.Time;
row_max = size(u.Data,2);
param_number = 5;
t_end = 1;
time_axis = [0 t_end];

fname1 = '/home/igricart/git/Tese_Mestrado/Simulink/noise_compTorque_HGO/figs';
fname2 = '/home/ignacio/gitGeneral/TeseMestrado/Simulink/noise_compTorque_HGO/figs';
if exist(fname1,'dir')==0
    if exist(fname2,'dir')==0
        error('Invalid path');
    else
        fname = fname2;
    end
else
    fname = fname1;
end

%% Error declaration

q_error = abs(q_ref.Data - q_hat.Data);
dq_error = abs(dq_ref.Data - dq_hat.Data);
q_est_error = abs(q.Data - q_hat.Data);
dq_est_error = abs(dq.Data - dq_hat.Data);

%% Define gain type

if Mu.Data(1) == Mu.Data(end)
    mu_type = 'fix';
else
    mu_type = 'var';
end

%% Units declaration

unit = [
    "(N.m)";
    "(m)";
    "(m/s)";
    "(rad)";
    "(rad/s)";
    "(N)";
    ];

%% Plot XY limits for Control Signal and Errors
%Control Signal (N.m)
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
f_u = figure();
grid on;

% for i=1:row_max
% % plot adaptated for u
% subplot(2,2,i);
% plot(t, u.Data(:,i));
% title(['Control Signal from joint' num2str(i) '(N.m)']);
% ylabel(['Control Signal (N.m)']);
% xlabel(['Time (sec)']);
% xlim(time_axis);
% ylim(Y_lim(i,1:2))
% saveas(f_u,fullfile(fname,['u_mu_' mu_type '_' num2str(sprintf('%.0d',Mu.Data(end)))]),'epsc');

% plot u
% subplot(param_number,row_max,i);
% plot(t, u.Data(:,i));
% title(['Control Signal from joint' num2str(i) '(N.m)']);
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
    -0.15 0.15; 
    -0.2 0.7; 
    -1.2 0.0; 
    -0.8 0.0];

% Joint and estimated Joint Velocity
y_lim_dq  = [
    -4 4; 
    -6 6; 
    -8 10; 
    -10 6];

Y2_lim = [y_lim_q, y_lim_dq];

%% Plot q q_hat dq dq_hat
% f2 = figure();
% grid on;
% 
% for i=1:row_max
% % plot q and q_hat
% subplot(2,row_max,i);
% plot(q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
% if (i == 1)
%     title(['Joint angle and Estimated' num2str(i) '(m)']);
% else
%    title(['Joint angle and Estimated' num2str(i) '(rad)']);
% end
% xlim(time_axis);
% ylim(Y2_lim(i,1:2));
% 
% % plot dq and dq_hat
% subplot(2,row_max,row_max + i);
% plot(dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
% if (i == 1)
%     title(['Joint velocity and Estimated' num2str(i) '(m/s)']);
% else
%    title(['Joint velocity and Estimated' num2str(i) '(rad/s)']);
% end
% xlim(time_axis);
% ylim(Y2_lim(i,3:4));
% 
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot 1 Article
% Joint and estimated joint angle on the left subplot and a zoomed plot on the
% right

% Joints zoomed
x_lim_q_zoom  = [
    0 0.02; 
    0 0.02; 
    0 0.02; 
    0 0.02];

y_lim_q_zoom  = [
    -0.0 0.04; 
    -0.0 1.0; 
    -0.19 -0.05; 
    -0.6 0.0];

X_lim_q_zoom = x_lim_q_zoom;
Y_lim_q_zoom = y_lim_q_zoom;


% Plot 1 - Hip Displacement
i = 1;
f_hip_q = figure();

subplot(1,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Hip displacement (m)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,1:2));

% Zoomed
subplot(1,2,2);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Hip displacement (m)']);
xlabel(['Time (sec)']);
xlim(X_lim_q_zoom(i,1:2));
ylim(Y_lim_q_zoom(i,1:2));
legend({'Desired','True','Estimated'},'Location','southeast');

set(f_hip_q,'units','pixels','position',[675,553,570,211]);
saveas(f_hip_q,fullfile(fname,['q_hip_mu_' mu_type '_' num2str(sprintf('%.0d',Mu.Data(end)))]),'epsc');

% Plot 2 - Thigh angle
% Thigh joint zoomed
i = 2;
f_thigh_q = figure();

subplot(1,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Thigh angle (rad)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,1:2));

% Zoomed
subplot(1,2,2);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Thigh angle (rad)']);
xlabel(['Time (sec)']);
xlim(X_lim_q_zoom(i,1:2));
ylim(Y_lim_q_zoom(i,1:2));
% legend({'Desired','True','Estimated'},'Location','southeast');

set(f_thigh_q,'units','pixels','position',[675,553,570,211]);
saveas(f_thigh_q,fullfile(fname,['q_thigh_mu_' mu_type '_' num2str(sprintf('%.0d',Mu.Data(end)))]),'epsc');

% Plot 3 - Knee angle
% Knee joint zoomed
i = 3;
f_knee_q = figure();

subplot(1,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Knee angle (rad)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,1:2));

% Zoomed
subplot(1,2,2);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Knee angle (rad)']);
xlabel(['Time (sec)']);
xlim(X_lim_q_zoom(i,1:2));
ylim(Y_lim_q_zoom(i,1:2));
% legend({'Desired','True','Estimated'},'Location','northeast');

set(f_knee_q,'units','pixels','position',[675,553,570,211]);
saveas(f_knee_q,fullfile(fname,['q_knee_mu_' mu_type '_' num2str(sprintf('%.0d',Mu.Data(end)))]),'epsc');

% Plot 4 - Ankle angle
% Ankle joint zoomed
i = 4;
f_ankle_q = figure();

subplot(1,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Ankle angle (rad)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,1:2));

% Zoomed
subplot(1,2,2);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Ankle angle (rad)']);
xlabel(['Time (sec)']);
xlim(X_lim_q_zoom(i,1:2));
ylim(Y_lim_q_zoom(i,1:2));
% legend({'Desired','True','Estimated'},'Location','northeast');

set(f_ankle_q,'units','pixels','position',[675,553,570,211]);
saveas(f_ankle_q,fullfile(fname,['q_ankle_mu_' mu_type '_' num2str(sprintf('%.0d',Mu.Data(end)))]),'epsc');

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
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Hip velocity (m/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));
legend({'True','Estimated'},'Location','southwest');

subplot(2,2,2);
i = 2;
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Thigh angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));
legend({'True','Estimated'},'Location','southwest');

subplot(2,2,3);
i = 3;
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Knee angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));
legend({'True','Estimated'},'Location','southwest');

subplot(2,2,4);
i = 4;
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
ylabel(['Ankle angular velocity (rad)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));
legend({'True','Estimated'},'Location','southwest');

saveas(f_joints_dq,fullfile(fname,['joint_velocities_mu_' mu_type '_' num2str(sprintf('%.0d',Mu.Data(end)))]),'epsc');

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
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Hip velocity (m/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));

% Zoomed
subplot(1,2,2);
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Hip velocity (m/s)']);
xlabel(['Time (sec)']);
xlim(X_lim_dq_zoom(i,1:2));
ylim(Y_lim_dq_zoom(i,1:2));
% legend({'Desired','True','Estimated'},'Location','northeast');

set(f_hip_dq,'units','pixels','position',[675,553,570,211]);
saveas(f_hip_dq,fullfile(fname,['dq_hip_mu_' mu_type '_' num2str(sprintf('%.0d',Mu.Data(end)))]),'epsc');

% Plot 2 - Thigh angular velocity
% Thigh joint zoomed
i = 2;
f_thigh_dq = figure();

subplot(1,2,1);
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Thigh angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));

% Zoomed
subplot(1,2,2);
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Thigh angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(X_lim_dq_zoom(i,1:2));
ylim(Y_lim_dq_zoom(i,1:2));
% legend({'Desired','True','Estimated'},'Location','southeast');

set(f_thigh_dq,'units','pixels','position',[675,553,570,211]);
saveas(f_thigh_dq,fullfile(fname,['dq_thigh_mu_' mu_type '_' num2str(sprintf('%.0d',Mu.Data(end)))]),'epsc');

% Plot 3 - Knee angular velocity
% Knee joint zoomed
i = 3;
f_knee_dq = figure();

subplot(1,2,1);
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Knee angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));

% Zoomed
subplot(1,2,2);
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Knee angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(X_lim_dq_zoom(i,1:2));
ylim(Y_lim_dq_zoom(i,1:2));
% legend({'Desired','True','Estimated'},'Location','southeast');

set(f_knee_dq,'units','pixels','position',[675,553,570,211]);
saveas(f_knee_dq,fullfile(fname,['dq_knee_mu_' mu_type '_' num2str(sprintf('%.0d',Mu.Data(end)))]),'epsc');

% Plot 4 - Ankle angle
% Ankle joint zoomed
i = 4;
f_ankle_dq = figure();

subplot(1,2,1);
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Ankle angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(Y2_lim(i,3:4));

% Zoomed
subplot(1,2,2);
plot(dq_ref.Time, dq_ref.Data(:,i),'k',dq.Time, dq.Data(:,i),dq_hat.Time, dq_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Ankle angular velocity (rad/s)']);
xlabel(['Time (sec)']);
xlim(X_lim_dq_zoom(i,1:2));
ylim(Y_lim_dq_zoom(i,1:2));
% legend({'Desired','True','Estimated'},'Location','northeast');

set(f_ankle_dq,'units','pixels','position',[675,553,570,211]);
saveas(f_ankle_dq,fullfile(fname,['dq_ankle_mu_' mu_type '_' num2str(sprintf('%.0d',Mu.Data(end)))]),'epsc');

%% Plot 4 Article
% Control signal and SNR plot

% Joints SNR
y_lim_q_SNR  = [
     0  400; 
     0  220; 
     0  100; 
     0  10;];

Y_lim_q_SNR = y_lim_q_SNR;


f_noise_comparison = figure();

% Plot 1 - Hip Noise
i = 1;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
subplot(2,2,i)
plot(SNR_used.Time, SNR_used.Data(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
plot(SNR_used.Time, SNR_used.Data(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
plot(SNR_used.Time, SNR_used.Data(:,i));
title('Hip');
grid on; grid minor;
ylabel(['Noise Energy']);
xlabel(['Time (sec)']);
%xlim(0:1);
ylim(Y_lim_q_SNR(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;

% Plot 2 - Thigh Noise
i = 2;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
subplot(2,2,i)
plot(SNR_used.Time, SNR_used.Data(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
plot(SNR_used.Time, SNR_used.Data(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
plot(SNR_used.Time, SNR_used.Data(:,i));
title('Thigh');
grid on; grid minor;
ylabel(['Noise Energy']);
xlabel(['Time (sec)']);
%xlim(0:1);
ylim(Y_lim_q_SNR(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;

% Plot 3 - Knee Noise
i = 3;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
subplot(2,2,i)
plot(SNR_used.Time, SNR_used.Data(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
plot(SNR_used.Time, SNR_used.Data(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
plot(SNR_used.Time, SNR_used.Data(:,i));
title('Knee');
grid on; grid minor;
ylabel(['Noise Energy']);
xlabel(['Time (sec)']);
%xlim(0:1);
ylim(Y_lim_q_SNR(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;

% Plot 4 - Ankle Noise
i = 4;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
subplot(2,2,i)
plot(SNR_used.Time, SNR_used.Data(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
plot(SNR_used.Time, SNR_used.Data(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
plot(SNR_used.Time, SNR_used.Data(:,i));
title('Ankle');
grid on; grid minor;
ylabel(['Noise Energy']);
xlabel(['Time (sec)']);
%xlim(0:1);
ylim(Y_lim_q_SNR(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;
saveas(f_noise_comparison,fullfile(fname,['SNR_comparison']),'epsc');

hold off;


%% Plot 5 Article
% Control signal plot

% u limits
y_lim_u  = [
    -800 800; 
    -800 800; 
    -300 300; 
    -60 60];

f_u_comparison = figure();

% Plot 1 - Hip u
i = 1;
load('protese_noise_PID_4e-4_HGOFeedback.mat');
subplot(2,2,i)
plot(u.Time, u.Data(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
plot(u.Time, u.Data(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
plot(u.Time, u.Data(:,i));
title('Hip');
grid on; grid minor;
ylabel(['Control Signal']);
xlabel(['Time (sec)']);
xlim([0 2]);
ylim(y_lim_u(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;

% Plot 2 - Thigh u
i = 2;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
subplot(2,2,i)
plot(u.Time, u.Data(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
plot(u.Time, u.Data(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
plot(u.Time, u.Data(:,i));
title('Thigh');
grid on; grid minor;
ylabel(['Control Signal']);
xlabel(['Time (sec)']);
xlim([0 2]);
ylim(y_lim_u(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;

% Plot 3 - Knee u
i = 3;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
subplot(2,2,i)
plot(u.Time, u.Data(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
plot(u.Time, u.Data(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
plot(u.Time, u.Data(:,i));
title('Knee');
grid on; grid minor;
ylabel(['Control Signal']);
xlabel(['Time (sec)']);
xlim([0 2]);
ylim(y_lim_u(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;

% Plot 4 - Ankle u
i = 4;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
subplot(2,2,i)
plot(u.Time, u.Data(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
plot(u.Time, u.Data(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
plot(u.Time, u.Data(:,i));
title('Ankle');
grid on; grid minor;
ylabel(['Control Signal']);
xlabel(['Time (sec)']);
xlim([0 2]);
ylim(y_lim_u(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;
saveas(f_u_comparison,fullfile(fname,['u_comparison']),'epsc');

hold off;

%% Plot 6 Article
% Mu signal plot

% u limits
y_lim_mu  = [
    0 20e-4];

f_mu_comparison = figure();
i = 1;
load('protese_noise_PID_4e-4_HGOFeedback.mat');
plot(Mu.Time, Mu.Data(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
plot(Mu.Time, Mu.Data(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
plot(Mu.Time, Mu.Data(:,i));
title('Mu signal');
grid on; grid minor;
% ylabel(['Mu Signal']);
xlabel(['Time (sec)']);
xlim(0:1);
ylim(y_lim_mu(i,1:2));
lgd = legend({'Static \mu_{1}','\mu_{var}','Static \mu_{2}'},'Location','northwest');
lgd.FontSize = 8;

set(f_mu_comparison,'units','pixels','position',[675,553,570,211]);
saveas(f_mu_comparison,fullfile(fname,['mu_comparison']),'epsc');

hold off;

%% Plot 7 Article
% Tracking error plot

% Tracking error (m or rad)
y_lim_q_error  = [
    0 0.02; 
    0 0.01; 
    0 0.015; 
    0 0.03];

f_q_error_comparison = figure();

% Plot 1 - Hip u
i = 1;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
subplot(2,2,i)
plot(q.Time, q_error(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
plot(q.Time, q_error(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
plot(q.Time, q_error(:,i));
title('Hip');
grid on; grid minor;
ylabel(['Tracking error']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(y_lim_q_error(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northeast');
% lgd.FontSize = 8;

% Plot 2 - Thigh u
i = 2;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
subplot(2,2,i)
plot(q.Time, q_error(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
plot(q.Time, q_error(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
plot(q.Time, q_error(:,i));
title('Thigh');
grid on; grid minor;
ylabel(['Tracking error']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(y_lim_q_error(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;

% Plot 3 - Knee u
i = 3;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
subplot(2,2,i)
plot(q.Time, q_error(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
plot(q.Time, q_error(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
plot(q.Time, q_error(:,i));
title('Knee');
grid on; grid minor;
ylabel(['Tracking error']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(y_lim_q_error(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;

% Plot 4 - Ankle u
i = 4;

load('protese_noise_PID_4e-4_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
subplot(2,2,i)
plot(q.Time, q_error(:,i));
hold on
load('protese_noise_PID_var_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
plot(q.Time, q_error(:,i));
load('protese_noise_PID_19e-4_HGOFeedback.mat');
q_error = abs(q_ref.Data - q_hat.Data);
plot(q.Time, q_error(:,i));
title('Ankle');
grid on; grid minor;
ylabel(['Tracking error']);
xlabel(['Time (sec)']);
xlim(time_axis);
ylim(y_lim_q_error(i,1:2));
% lgd = legend({'4e-4','Variable','19e-4'},'Location','northwest');
% lgd.FontSize = 8;
saveas(f_q_error_comparison,fullfile(fname,['q_error_comparison']),'epsc');

hold off;
% close all;
% subplot(3,3,1)
% plot(u.Time, u.Data(:,i));
% grid on; grid minor;
% ylabel(['Hip control signal']);
% xlabel(['Time (sec)']);
% xlim(time_axis);
% ylim(Y_lim(i,1:2));
% 
% subplot(3,3,4)
% plot(q_ref.Time, q_error(:,i));
% grid on; grid minor;
% ylabel(['Hip tracking error']);
% xlabel(['Time (sec)']);
% xlim(time_axis);
% ylim(Y_lim(i,3:4));
% 
% subplot(3,3,7)
% plot(SNR_used.Time, SNR_used.Data(:,i));
% grid on; grid minor;
% ylabel(['Hip tracking error']);
% xlabel(['Time (sec)']);
% xlim(time_axis);
% ylim(Y_lim_q_SNR(i,1:2));f_trecking_error