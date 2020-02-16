% Run script
run Init_sim.m;
run Init_HGO.m;

param_error = 1.0406;
Mass_ctrl = Mass_ctrl.*param_error;
Inertia_ctrl = Inertia_ctrl.*param_error;
R_ctrl = R_ctrl.*param_error;
L_ctrl = L_ctrl.*param_error;
h_ctrl = h_ctrl.*param_error;

wn = 2*pi*4;
zeta = 0.9;
p = 1.5*wn;
ki = p*wn^2;
kp = 2*zeta*wn*p + wn^2;
kd = p + 2*zeta*wn;
Kp = kp*eye(n); Kd = kd*eye(n); Ki = ki*eye(n);

converter = 180/pi;

%% Run simulation
% Noise k_noise = [Noise1 Noise2 ... NoiseN]
k_noise = [2e-7 1e-06];
j = 1;

for i=1:size(k_noise,2)
    % HGO 4e-4
    Mu_hgo = 4e-4;
    Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
    sim('simulation_hgo2017a',1);
    save(['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '.mat']);

    % HGO 10e-4
    Mu_hgo = 10e-4;
    Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
    sim('simulation_hgo2017a',1);
    save(['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '.mat']);

    % HGO 19e-4
    Mu_hgo = 19e-4;
    Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
    sim('simulation_hgo2017a',1);
    save(['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '.mat']);
end

%% Plot

fname1 = '/home/ignacio/Documents/Msc/Tese_Mestrado/Simulink/general_sim/figs';
fname2 = '/home/igricart/Documents/Tese_Mestrado/Simulink/general_sim/figs';
if exist(fname1,'dir')==0
    if exist(fname2,'dir')==0
        error('Invalid path');
    else
        fname = fname2;
    end
else
    fname = fname1;
end
% Plot Joints
%----------------------------------------------------------------
% Plot 1.1 Joints

i = 1;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
f_hip = figure('Name', 'Hip displacement');
subplot(3,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Hip vertical displacement (m)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,2);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);
legend({'Desired','True','Estimated'},'Location','southeast');

load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
subplot(3,2,3);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Hip vertical displacement (m)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,4);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);

load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
subplot(3,2,5);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Hip vertical displacement (m)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,6);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);

set(f_hip,'units','pixels','position',[900,900,900,900]);
saveas(f_hip,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_hip']),'epsc');


%-----------------------------------------------------------------------------------------------------------------
% Plot 1.2 States tracking and estimation errors

f_hip_error = figure('Name', 'Hip displacement error and control signal');
i = 1;

subplot(2, 2, 1)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i))
xlabel(['Time(s)']);
ylabel(['e (m)']);

subplot(2, 2, 2)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i))
xlabel(['Time(s)']);
ylabel('$\dot{e} (m)$', 'Interpreter','latex');

subplot(2, 2, 3)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i))
xlabel(['Time(s)']);
ylabel(['\zeta (m)']);

subplot(2, 2, 4)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i))
xlabel(['Time(s)']);
ylabel('$\dot{\zeta} (m)$', 'Interpreter','latex');

set(f_hip_error,'units','pixels','position',[900,900,900,900]);
saveas(f_hip_error,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_hip']),'epsc');

%-------------------------------------------------------------------------------------------------------------
% Plot 1.3 Control Signal

f_hip_u = figure('Name', 'Hip control signal');

subplot(1,2,1)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
xlabel(['Time (s)']);
ylabel(['u (N)']);

subplot(1,2,2)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
ylim([-1000 400])
xlabel(['Time (s)']);
ylabel(['u (N)']);

set(f_hip_u,'units','pixels','position',[900,900,900,450]);
saveas(f_hip_u,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_hip']),'epsc');

%---------------------------------------------------------------------------------------------------------------
% Plot 2.1 Joints

i = 2;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
f_thigh = figure('Name', 'Hip angle');
subplot(3,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Hip angle (deg)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,2);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);
legend({'Desired','True','Estimated'},'Location','southeast');

load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
subplot(3,2,3);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Hip angle (deg)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,4);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);

load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
subplot(3,2,5);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Hip angle (deg)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,6);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);

set(f_thigh,'units','pixels','position',[900,900,900,900]);
saveas(f_thigh,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_thigh']),'epsc');

%-----------------------------------------------------------------------------------------------------------------
% Plot 2.2 States tracking and estimation errors

f_thigh_error = figure('Name', 'Hip angle error and control signal');

subplot(2, 2, 1)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel(['e (deg)']);

subplot(2, 2, 2)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel('$\dot{e} (deg/s)$', 'Interpreter','latex');

subplot(2, 2, 3)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel(['\zeta (deg)']);

subplot(2, 2, 4)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel('$\dot{\zeta} (deg/s)$', 'Interpreter','latex');

set(f_thigh_error,'units','pixels','position',[900,900,900,900]);
saveas(f_thigh_error,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_thigh']),'epsc');

%-------------------------------------------------------------------------------------------------------------
% Plot 2.3 Control Signal

f_thigh_u = figure('Name', 'Hip control signal');

subplot(1,2,1)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
xlabel(['Time (s)']);
ylabel(['u (Nm)']);

subplot(1,2,2)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
ylim([-300 400])
xlabel(['Time (s)']);
ylabel(['u (Nm)']);

set(f_thigh_u,'units','pixels','position',[900,900,900,450]);
saveas(f_thigh_u,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_thigh']),'epsc');

%---------------------------------------------------------------------------------------------------------------
% Plot 3.1 Joints

i = 3;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
f_knee = figure('Name', 'Knee angle');
subplot(3,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Knee angle (deg)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,2);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);
legend({'Desired','True','Estimated'},'Location','southeast');

load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
subplot(3,2,3);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Knee angle (deg)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,4);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);

load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
subplot(3,2,5);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Knee angle (deg)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,6);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);

set(f_knee,'units','pixels','position',[900,900,900,900]);
saveas(f_knee,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_knee']),'epsc');

%-----------------------------------------------------------------------------------------------------------------
% Plot 3.2 States tracking and estimation errors

f_knee_error = figure('Name', 'Knee angle error and control signal');

subplot(2, 2, 1)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel(['e (deg)']);

subplot(2, 2, 2)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel('$\dot{e} (deg/s)$', 'Interpreter','latex');

subplot(2, 2, 3)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel(['\zeta (deg)']);

subplot(2, 2, 4)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel('$\dot{\zeta} (deg/s)$', 'Interpreter','latex');

set(f_knee_error,'units','pixels','position',[900,900,900,900]);
saveas(f_knee_error,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_knee']),'epsc');
%-------------------------------------------------------------------------------------------------------------
% Plot 3.3 Control Signal

f_knee_u = figure('Name', 'Knee control signal');

subplot(1,2,1)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
xlabel(['Time (s)']);
ylabel(['u (Nm)']);

subplot(1,2,2)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
ylim([-100 100])
xlabel(['Time (s)']);
ylabel(['u (Nm)']);

set(f_knee_u,'units','pixels','position',[900,900,900,450]);
saveas(f_knee_u,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_knee']),'epsc');

%---------------------------------------------------------------------------------------------------------------
% Plot 4.1 Joints

i = 4;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
f_ankle = figure('Name', 'Ankle angle');
subplot(3,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Ankle angle (deg)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,2);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);
legend({'Desired','True','Estimated'},'Location','southeast');

load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
subplot(3,2,3);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Ankle angle (deg)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,4);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);

load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
subplot(3,2,5);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
ylabel(['Ankle angle (deg)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(3,2,6);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i),q_hat.Time, q_hat.Data(:,i),'--');
grid on; grid minor;
xlabel(['Time (sec)']);
xlim([0 0.05]);

set(f_ankle,'units','pixels','position',[900,900,900,900]);
saveas(f_ankle,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_ankle']),'epsc');

%-----------------------------------------------------------------------------------------------------------------
% Plot 4.2 States tracking and estimation errors

f_ankle_error = figure('Name', 'Ankle angle error and control signal');

subplot(2, 2, 1)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(q_error.Time, q_error.Data(:,i)*converter)
xlabel(['time (s)'])
ylabel(['e (deg)']);

subplot(2, 2, 2)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(dq_error.Time, dq_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel('$\dot{e} (deg/s)$', 'Interpreter','latex');

subplot(2, 2, 3)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(q_est_error.Time, q_est_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel(['\zeta (deg)']);

subplot(2, 2, 4)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i)*converter)
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i)*converter)
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(dq_est_error.Time, dq_est_error.Data(:,i)*converter)
xlabel(['Time(s)']);
ylabel('$\dot{\zeta} (deg/s)$', 'Interpreter','latex');

set(f_ankle_error,'units','pixels','position',[900,900,900,900]);
saveas(f_ankle_error,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_ankle']),'epsc');

%-------------------------------------------------------------------------------------------------------------
% Plot 4.3 Control Signal

f_ankle_u = figure('Name', 'Ankle control signal');

subplot(1,2,1)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
xlabel(['Time (s)']);
ylabel(['u (Nm)']);

subplot(1,2,2)
hold on;
grid on; grid minor;
load(['output_feedback_0.0004_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.001_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
load(['output_feedback_0.0019_' num2str(k_noise(j)) '.mat']);
plot(u.Time, u.Data(:,i))
ylim([-15 15])
xlabel(['Time (s)']);
ylabel(['u (Nm)']);

set(f_ankle_u,'units','pixels','position',[900,900,900,450]);
saveas(f_ankle_u,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise(j)) '_ankle']),'epsc');