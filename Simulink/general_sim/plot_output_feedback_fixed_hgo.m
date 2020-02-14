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
% Noise 2e-7
k_noise = 2e-7;

% HGO 4e-4
Mu_hgo = 4e-4;
Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
sim('simulation_hgo2017a',1);
save(['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '.mat']);

% HGO 10e-4
Mu_hgo = 10e-4;
Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
sim('simulation_hgo2017a',1);
save(['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '.mat']);

% HGO 19e-4
Mu_hgo = 19e-4;
Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
sim('simulation_hgo2017a',1);
save(['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '.mat']);

% Noise 1e-6
k_noise = 1e-6;

% HGO 4e-4
Mu_hgo = 4e-4;
Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
sim('simulation_hgo2017a',1);
save(['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '.mat']);

% HGO 10e-4
Mu_hgo = 10e-4;
Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
sim('simulation_hgo2017a',1);
save(['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '.mat']);

% HGO 19e-4
Mu_hgo = 19e-4;
Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
sim('simulation_hgo2017a',1);
save(['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '.mat']);

%% Plot
fname = '/home/ignacio/Documents/Msc/Tese_Mestrado/Simulink/general_sim/figs';
    % Plot Joints
    % Plot 1 - Hip Displacement
    i = 1;
    % Plot 1.1 Joints
    load output_feedback_0.0004_2e-07.mat;
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
    
    load output_feedback_0.001_2e-07.mat;
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

    load output_feedback_0.0019_2e-07.mat;
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
    saveas(f_hip,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '_hip']),'epsc');


    %-----------------------------------------------------------------------------------------------------------------
    % Plot 1.2 Control Signal Error
    
    f_hip_u_error = figure('Name', 'Hip displacement error and control signal');
    
    
    subplot(2,2,1)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_2e-07.mat
    plot(q_error.Time, q_error.Data(:,1))
    load output_feedback_0.001_2e-07.mat
    plot(q_error.Time, q_error.Data(:,1))
    load output_feedback_0.0019_2e-07.mat
    plot(q_error.Time, q_error.Data(:,1))
    
    subplot(2,2,2)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_2e-07.mat
    plot(u.Time, u.Data(:,1))
    load output_feedback_0.001_2e-07.mat
    plot(u.Time, u.Data(:,1))
    load output_feedback_0.0019_2e-07.mat
    plot(u.Time, u.Data(:,1))
    
    subplot(2,2,3)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_1e-06.mat
    plot(q_error.Time, q_error.Data(:,1))
    load output_feedback_0.001_1e-06.mat
    plot(q_error.Time, q_error.Data(:,1))
    load output_feedback_0.0019_1e-06.mat
    plot(q_error.Time, q_error.Data(:,1))
    
    subplot(2,2,4)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_1e-06.mat
    plot(u.Time, u.Data(:,1))
    load output_feedback_0.001_1e-06.mat
    plot(u.Time, u.Data(:,1))
    load output_feedback_0.0019_1e-06.mat
    plot(u.Time, u.Data(:,1))

    set(f_hip_u_error,'units','pixels','position',[900,900,900,900]);
    saveas(f_hip_u_error,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '_hip']),'epsc');

    %---------------------------------------------------------------------------------------------------------------
    % Plot 2.1 Joints
    i = 2;
    load output_feedback_0.0004_2e-07.mat;
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
    
    load output_feedback_0.001_2e-07.mat;
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

    load output_feedback_0.0019_2e-07.mat;
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
    saveas(f_thigh,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '_thigh']),'epsc');

    %-----------------------------------------------------------------------------------------------------------------
    % Plot 2.2 Control Signal Error
    i = 2;
    f_thigh_u_error = figure('Name', 'Hip angle error and control signal');
    
    
    subplot(2,2,1)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_2e-07.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.001_2e-07.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.0019_2e-07.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    
    subplot(2,2,2)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_2e-07.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.001_2e-07.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.0019_2e-07.mat
    plot(u.Time, u.Data(:,i))
    
    subplot(2,2,3)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_1e-06.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.001_1e-06.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.0019_1e-06.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    
    subplot(2,2,4)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_1e-06.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.001_1e-06.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.0019_1e-06.mat
    plot(u.Time, u.Data(:,i))

    set(f_thigh_u_error,'units','pixels','position',[900,900,900,900]);
    saveas(f_thigh_u_error,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '_thigh']),'epsc');

    %---------------------------------------------------------------------------------------------------------------
    % Plot 3.1 Joints
    i = 3;
    load output_feedback_0.0004_2e-07.mat;
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
    
    load output_feedback_0.001_2e-07.mat;
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

    load output_feedback_0.0019_2e-07.mat;
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
    saveas(f_knee,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '_knee']),'epsc');

    %-----------------------------------------------------------------------------------------------------------------
    % Plot 3.2 Control Signal Error
    i = 3;
    f_knee_u_error = figure('Name', 'Knee angle error and control signal');
    
    
    subplot(2,2,1)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_2e-07.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.001_2e-07.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.0019_2e-07.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    
    subplot(2,2,2)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_2e-07.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.001_2e-07.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.0019_2e-07.mat
    plot(u.Time, u.Data(:,i))
    
    subplot(2,2,3)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_1e-06.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.001_1e-06.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.0019_1e-06.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    
    subplot(2,2,4)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_1e-06.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.001_1e-06.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.0019_1e-06.mat
    plot(u.Time, u.Data(:,i))

    set(f_knee_u_error,'units','pixels','position',[900,900,900,900]);
    saveas(f_knee_u_error,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '_knee']),'epsc');

    %---------------------------------------------------------------------------------------------------------------
    % Plot 4.1 Joints
    i = 4;
    load output_feedback_0.0004_2e-07.mat;
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
    
    load output_feedback_0.001_2e-07.mat;
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

    load output_feedback_0.0019_2e-07.mat;
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
    saveas(f_ankle,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '_ankle']),'epsc');

    %-----------------------------------------------------------------------------------------------------------------
    % Plot 4.2 Control Signal Error
    i = 4;
    f_ankle_u_error = figure('Name', 'Ankle angle error and control signal');
    
    
    subplot(2,2,1)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_2e-07.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.001_2e-07.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.0019_2e-07.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    
    subplot(2,2,2)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_2e-07.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.001_2e-07.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.0019_2e-07.mat
    plot(u.Time, u.Data(:,i))
    
    subplot(2,2,3)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_1e-06.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.001_1e-06.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    load output_feedback_0.0019_1e-06.mat
    plot(q_error.Time, q_error.Data(:,i)*converter)
    
    subplot(2,2,4)
    hold on;
    grid on; grid minor;
    load output_feedback_0.0004_1e-06.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.001_1e-06.mat
    plot(u.Time, u.Data(:,i))
    load output_feedback_0.0019_1e-06.mat
    plot(u.Time, u.Data(:,i))

    set(f_ankle_u_error,'units','pixels','position',[900,900,900,900]);
    saveas(f_ankle_u_error,fullfile(fname,['output_feedback_' num2str(Mu_hgo) '_' num2str(k_noise) '_ankle']),'epsc');

    