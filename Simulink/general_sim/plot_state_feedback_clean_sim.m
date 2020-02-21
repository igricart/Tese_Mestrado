% Run script
run Init_sim.m;

param_error = 1;
Mass_ctrl = Mass_ctrl.*param_error;
Inertia_ctrl = Inertia_ctrl.*param_error;
R_ctrl = R_ctrl.*param_error;
L_ctrl = L_ctrl.*param_error;
h_ctrl = h_ctrl.*param_error;

k_noise = 0;    

wn = 2*pi*4;
zeta = 0.9;
p = 1.5*wn;
ki = p*wn^2;
kp = 2*zeta*wn*p + wn^2;
kd = p + 2*zeta*wn;
Kp = kp*eye(n); Kd = kd*eye(n); Ki = ki*eye(n);

% Run simulation
sim('simulation_state2017a',1);
save('state_feedback_clean.mat');
%% Plot
fname = '/home/ignacio/Documents/Msc/Tese_Mestrado/Simulink/general_sim/figs';
    
    converter = 180/pi;
    % Plot Joints
    % Plot 1 - Hip Displacement
    i = 1;
    f_hip = figure('Name', 'Hip displacement');

    % Plot q
    subplot(2,2,1);
    title('(a)', 'FontSize', 10);
    hold on;
    plot(q_ref.Time, q_ref.Data(:,i), q.Time, q.Data(:,i));
    grid on; grid minor;
    ylabel(['Displacement (m)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','northeast');

    % Plot dq
    subplot(2,2,2);
    title('(b)', 'FontSize', 10);
    hold on;
    plot(dq_ref.Time, dq_ref.Data(:,i), dq.Time, dq.Data(:,i));
    grid on; grid minor;
    ylabel(['Velocity (m/s)']);
    xlabel(['Time (sec)']);
    
    % Plot tau
    subplot(2,2,3);
    title('(c)', 'FontSize', 10);
    hold on;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Force (N)']);
    xlabel(['Time (sec)']);

    % Plot error
    subplot(2,2,4);
    title('(d)', 'FontSize', 10);
    hold on;
    plot(q_error.Time, q_error.Data(:,1));
    grid on; grid minor;
    ylabel(['Error (m)']);
    xlabel(['Time (sec)']);
    
    set(f_hip,'units','pixels','position',[900,900,900,900]);
    saveas(f_hip,fullfile(fname,['state_feedback_clean_hip']),'epsc');

    % Plot 2 - Thigh angle
    % Thigh joint zoomed
    i = 2;
    f_thigh = figure('Name', 'Hip Extension/Flexion');

    % Plot q
    subplot(2,2,1);
    title('(a)', 'FontSize', 10);
    hold on;
    plot(q_ref.Time, q_ref.Data(:,i)*converter, q.Time, q.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Angle (deg)']);
    xlabel(['Time (sec)']);

    % Plot dq
    subplot(2,2,2);
    title('(b)', 'FontSize', 10);
    hold on;
    plot(dq_ref.Time, dq_ref.Data(:,i)*converter, dq.Time, dq.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Velocity (deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');
    
    % Plot tau
    subplot(2,2,3);
    title('(c)', 'FontSize', 10);
    hold on;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Torque (N.m)']);
    xlabel(['Time (sec)']);

    % Plot error
    subplot(2,2,4);
    title('(d)', 'FontSize', 10);
    hold on;
    plot(q_error.Time, q_error.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Error (deg)']);
    xlabel(['Time (sec)']);

    set(f_thigh,'units','pixels','position',[900,900,900,900]);
    saveas(f_thigh,fullfile(fname,['state_feedback_clean_thigh']),'epsc');

    % Plot 3 - Knee angle
    % Knee joint zoomed
    i = 3;
    f_knee = figure('Name', 'Knee angle');

    % Plot q
    subplot(2,2,1);
    title('(a)', 'FontSize', 10);
    hold on;
    plot(q_ref.Time, q_ref.Data(:,i)*converter, q.Time, q.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Angle (deg)']);
    xlabel(['Time (sec)']);

    % Plot dq
    subplot(2,2,2);
    title('(b)', 'FontSize', 10);
    hold on;
    plot(dq_ref.Time, dq_ref.Data(:,i)*converter, dq.Time, dq.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Velocity (deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');
    
    % Plot tau
    subplot(2,2,3);
    title('(c)', 'FontSize', 10);
    hold on;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Torque (N.m)']);
    xlabel(['Time (sec)']);

    % Plot error
    subplot(2,2,4);
    title('(d)', 'FontSize', 10);
    hold on;
    plot(q_error.Time, q_error.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Error (deg)']);
    xlabel(['Time (sec)']);

    set(f_knee,'units','pixels','position',[900,900,900,900]);
    saveas(f_knee,fullfile(fname,['state_feedback_clean_knee']),'epsc');

    % Plot 4 - Ankle angle
    % Ankle joint zoomed
    i = 4;
    f_ankle = figure('Name', 'Ankle angle');

    % Plot q
    subplot(2,2,1);
    title('(a)', 'FontSize', 10);
    hold on;
    plot(q_ref.Time, q_ref.Data(:,i)*converter, q.Time, q.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Angle (deg)']);
    xlabel(['Time (sec)']);

    % Plot dq
    subplot(2,2,2);
    title('(b)', 'FontSize', 10);
    hold on;
    plot(dq_ref.Time, dq_ref.Data(:,i)*converter, dq.Time, dq.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Velocity (deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');
    
    % Plot tau
    subplot(2,2,3);
    title('(c)', 'FontSize', 10);
    hold on;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Torque (N.m)']);
    xlabel(['Time (sec)']);

    % Plot error
    subplot(2,2,4);
    title('(d)', 'FontSize', 10);
    hold on;
    plot(q_error.Time, q_error.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Error (deg)']);
    xlabel(['Time (sec)']);

    set(f_ankle,'units','pixels','position',[900,900,900,900]);
    saveas(f_ankle,fullfile(fname,['state_feedback_clean_ankle']),'epsc');
