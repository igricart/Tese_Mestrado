%% Initialize simulation parameters
% PLOT_FIGURES Use:
% arg1 as control ('PD', 'PID')
% arg2 as mode ('weak', 'medium', 'strong')
% arg3 as initial state ('correct' or 'zero')
% arg4 as uncertainty in decimal (DOUBLE)
% arg5 as the noise added to the system (DOUBLE)

% Run script
run Init_sim.m;

% hgo: 1 - output feedback, 0 - state feedback
% is: 'correct', 'zero'
% control: 'PID', 'PD'
% mode: 'weak', 'medium', 'strong'
% uncert: 0.value
% k_noise: 0.value

hgo = 1;
is = 'correct';
hgo_is = 'zero';
control = 'PID';
mode = 'strong';
uncert = 0;
k_noise = 2e-7;

% Gain
Mu_hgo = 4e-4;
mu = num2str(Mu_hgo);
Hmu = [eye(4)/Mu zeros(4); zeros(4) eye(4)/Mu^2];

% Initial state
if strcmp(is,'correct')
    % Correct
    qInit = [ 0.0216 0.5675 -0.13 -0.39 ]';
elseif strcmp(is,'zero')
    % Zero
    qInit = [0 0 0 0]';
end

% HGO Initial state
if strcmp(hgo_is,'correct')
    % Correct
    q_hat_init = [ 0.0216 0.5675 -0.13 -0.39 ]';
elseif strcmp(hgo_is,'zero')
    % Zero
    q_hat_init = [0 0 0 0]';
end

% Set different control gains
if strcmp(control,'PD')
    if strcmp(mode,'weak')
        wn = 2*pi;
        zeta = 0.85;
    elseif strcmp(mode,'medium')
        wn = 2*pi*2;
        zeta = 0.9;
    elseif strcmp(mode,'strong')
        wn = 2*pi*2;
        zeta = 1;
    end
    kd = 2*zeta*wn;
    ki = 0;
    kp = wn^2;
    Kp = kp*eye(n); Kd = kd*eye(n); Ki = ki*eye(n);
elseif strcmp(control,'PID')
    if strcmp(mode,'weak')
        wn = pi;
        zeta = 0.8;
        p = wn;
    elseif strcmp(mode,'medium')
        wn = 2*pi*3;
        zeta = 0.9;
        p = 2*wn;
    elseif strcmp(mode,'strong')
        wn = 2*pi*8;
        zeta = 0.9;
        p = 2*wn;
    end
    ki = p*wn^2;
    kp = 2*zeta*wn*p + wn^2;
    kd = p + 2*zeta*wn;
    Kp = kp*eye(n); Kd = kd*eye(n); Ki = ki*eye(n);
end

% Add uncertainties
uncertainty = num2str(uncert);

% Estimated Inertia
Inertia_ctrl = Inertia + [zeros(size(Inertia,1),2) uncert*(rand(size(Inertia(:,3:end)))-0.5*ones(size(Inertia(:,3:end))))];

% Estimated C.M
R_ctrl = R + [zeros(size(R,1),2) uncert*(rand(size(R(:,3:end)))-0.5*ones(size(R(:,3:end))))];

% Axis misalignment
%h_ctrl = h_ctrl.*param_error;

% Add noise
noise = num2str(k_noise);

% Run simulation
sim('simulation',2)

% Useful set function
%set(f_hip_q,'units','pixels','position',[675,553,570,211]);

%% State Feedback
fname = '/home/ignacio/Documents/Msc/Tese_Mestrado/Simulink/general_sim/figs';
for i = 1:2
    if i == 1
        load('/home/igricart/Documents/Tese_Mestrado/Simulink/general_sim/mat_files/acc_state_feedback_clean.mat');
    elseif i == 2
        load('/home/igricart/Documents/Tese_Mestrado/Simulink/general_sim/mat_files/acc_state_feedback_noise_param_error.mat');
    end
    
    converter = 180/pi;
    % Plot Joints
    % Plot 1 - Hip Displacement and Vel
    i = 1;
    f_hip = figure('Name', 'Hip displacement');

    subplot(1,2,1);
    plot(q_ref.Time, q_ref.Data(:,i), q.Time, q.Data(:,i));
    grid on; grid minor;
    ylabel(['Hip vertical displacement (m)']);
    xlabel(['Time (sec)']);

    % Zoomed
    subplot(1,2,2);
    %plot(dq_ref.Time, dq_ref.Data(:,i),'k.', dq.Time, dq.Data(:,i),'MarkerIndices',1:300:length(q_ref.Data(:,i)));
    plot(dq_ref.Time, dq_ref.Data(:,i), dq.Time, dq.Data(:,i));
    grid on; grid minor;
    ylabel(['Hip vel (m/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');

    %saveas(f_hip,fullfile(fname,['hip_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');

    % Plot 2 - Thigh angle
    % Thigh joint zoomed
    i = 2;
    f_thigh = figure();

    subplot(1,2,1);
    plot(q_ref.Time, q_ref.Data(:,i)*converter, q.Time, q.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Thing angle (deg)']);
    xlabel(['Time (sec)']);

    % Zoomed
    subplot(1,2,2);
    plot(dq_ref.Time, dq_ref.Data(:,i)*converter, dq.Time, dq.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Thigh angular vel (deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');

    %saveas(f_thigh,fullfile(fname,['thigh_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


    % Plot 3 - Knee angle
    % Knee joint zoomed
    i = 3;
    f_knee = figure();

    subplot(1,2,1);
    plot(q_ref.Time, q_ref.Data(:,i)*converter, q.Time, q.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Knee angle (deg)']);
    xlabel(['Time (sec)']);

    % Zoomed
    subplot(1,2,2);
    plot(dq_ref.Time, dq_ref.Data(:,i)*converter, dq.Time, dq.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Knee angular vel (deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');

    %saveas(f_knee,fullfile(fname,['knee_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


    % Plot 4 - Ankle angle
    % Ankle joint zoomed
    i = 4;
    f_ankle = figure();

    subplot(1,2,1);
    plot(q_ref.Time, q_ref.Data(:,i)*converter, q.Time, q.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Ankle angle (deg)']);
    xlabel(['Time (sec)']);

    % Zoomed
    subplot(1,2,2);
    plot(dq_ref.Time, dq_ref.Data(:,i)*converter, dq.Time, dq.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Ankle angular vel (deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');

    %saveas(f_ankle,fullfile(fname,['ankle_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


    % Plot Error

    % JJ Jeral Junto

    f_error = figure();
    plot(q_error.Time, q_error.Data);
    %set(f_error,'units','pixels','position',[675,553,570,211]);
    grid on; grid minor;
    ylabel(['Joint Error (m rad rad rad)']);
    xlabel(['Time (sec)']);

    %saveas(f_error,fullfile(fname,['error_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');

    % JJ Jeral Junto DEGREE

    f_error_deg = figure();
    plot(q_error.Time, q_error.Data(:,1), q_error.Time, q_error.Data(:,2:4)*converter);
    %set(f_error,'units','pixels','position',[675,553,570,211]);
    grid on; grid minor;
    ylabel(['Joint Error (m deg deg deg)']);
    xlabel(['Time (sec)']);
    legend({'Joint1','Joint2','Joint3','Joint4'},'Location','northeast');
    %saveas(f_error_deg,fullfile(fname,['error_control_state_feedback_deg_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');

    f_d_error_deg = figure();
    plot(q_error.Time, dq_error.Data(:,1)*converter, q_error.Time, dq_error.Data(:,2:4)*converter);
    %set(f_error,'units','pixels','position',[675,553,570,211]);
    grid on; grid minor;
    ylabel(['Joint Error (m/s deg/s deg/s deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Joint1','Joint2','Joint3','Joint4'},'Location','northeast');

    %saveas(f_d_error_deg,fullfile(fname,['error_control_state_feedback_deg_dq_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');

    % Plot Control Signal

    f_control = figure();

    subplot(2,2,1);
    i = 1;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Hip linear control signal (N)']);
    xlabel(['Time (sec)']);

    subplot(2,2,2);
    i = 2;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Hip angular control signal (N.m)']);
    xlabel(['Time (sec)']);

    subplot(2,2,3);
    i = 3;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Knee control signal (N.m)']);
    xlabel(['Time (sec)']);

    subplot(2,2,4);
    i = 4;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Ankle control signal (N.m)']);
    xlabel(['Time (sec)']);
end

%% HGO State Feedback
fname = '/home/ignacio/Documents/Msc/Tese_Mestrado/Simulink/general_sim/figs';
for i = 1:3
    if i == 1
        load('/home/igricart/Documents/Tese_Mestrado/Simulink/general_sim/mat_files/acc_mu_4e-4.mat');
    elseif i == 2
        load('/home/igricart/Documents/Tese_Mestrado/Simulink/general_sim/mat_files/acc_mu_10e-4.mat');
    elseif i == 3
        load('/home/igricart/Documents/Tese_Mestrado/Simulink/general_sim/mat_files/acc_mu_19e-4.mat');
    end
    
    converter = 180/pi;
    % Plot Joints
    % Plot 1 - Hip Displacement and Vel
    i = 1;
    f_hip = figure('Name', 'Hip displacement');

    subplot(1,2,1);
    plot(q_ref.Time, q_ref.Data(:,i), q.Time, q.Data(:,i));
    grid on; grid minor;
    ylabel(['Hip vertical displacement (m)']);
    xlabel(['Time (sec)']);

    % Zoomed
    subplot(1,2,2);
    %plot(dq_ref.Time, dq_ref.Data(:,i),'k.', dq.Time, dq.Data(:,i),'MarkerIndices',1:300:length(q_ref.Data(:,i)));
    plot(dq_ref.Time, dq_ref.Data(:,i), dq.Time, dq.Data(:,i));
    grid on; grid minor;
    ylabel(['Hip vel (m/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');

    %saveas(f_hip,fullfile(fname,['hip_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');

    % Plot 2 - Thigh angle
    % Thigh joint zoomed
    i = 2;
    f_thigh = figure();

    subplot(1,2,1);
    plot(q_ref.Time, q_ref.Data(:,i)*converter, q.Time, q.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Thing angle (deg)']);
    xlabel(['Time (sec)']);

    % Zoomed
    subplot(1,2,2);
    plot(dq_ref.Time, dq_ref.Data(:,i)*converter, dq.Time, dq.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Thigh angular vel (deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');

    %saveas(f_thigh,fullfile(fname,['thigh_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


    % Plot 3 - Knee angle
    % Knee joint zoomed
    i = 3;
    f_knee = figure();

    subplot(1,2,1);
    plot(q_ref.Time, q_ref.Data(:,i)*converter, q.Time, q.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Knee angle (deg)']);
    xlabel(['Time (sec)']);

    % Zoomed
    subplot(1,2,2);
    plot(dq_ref.Time, dq_ref.Data(:,i)*converter, dq.Time, dq.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Knee angular vel (deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');

    %saveas(f_knee,fullfile(fname,['knee_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


    % Plot 4 - Ankle angle
    % Ankle joint zoomed
    i = 4;
    f_ankle = figure();

    subplot(1,2,1);
    plot(q_ref.Time, q_ref.Data(:,i)*converter, q.Time, q.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Ankle angle (deg)']);
    xlabel(['Time (sec)']);

    % Zoomed
    subplot(1,2,2);
    plot(dq_ref.Time, dq_ref.Data(:,i)*converter, dq.Time, dq.Data(:,i)*converter);
    grid on; grid minor;
    ylabel(['Ankle angular vel (deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Desired','True'},'Location','southeast');

    %saveas(f_ankle,fullfile(fname,['ankle_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


    % Plot Error

    % JJ Jeral Junto

    f_error = figure();
    plot(q_error.Time, q_error.Data);
    %set(f_error,'units','pixels','position',[675,553,570,211]);
    grid on; grid minor;
    ylabel(['Joint Error (m rad rad rad)']);
    xlabel(['Time (sec)']);

    %saveas(f_error,fullfile(fname,['error_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');

    % JJ Jeral Junto DEGREE

    f_error_deg = figure();
    plot(q_error.Time, q_error.Data(:,1), q_error.Time, q_error.Data(:,2:4)*180/pi);
    %set(f_error,'units','pixels','position',[675,553,570,211]);
    grid on; grid minor;
    ylabel(['Joint Error (m deg deg deg)']);
    xlabel(['Time (sec)']);
    legend({'Joint1','Joint2','Joint3','Joint4'},'Location','northeast');
    %saveas(f_error_deg,fullfile(fname,['error_control_state_feedback_deg_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');

    f_d_error_deg = figure();
    plot(q_error.Time, dq_error.Data(:,1)*180/pi, q_error.Time, dq_error.Data(:,2:4)*180/pi);
    %set(f_error,'units','pixels','position',[675,553,570,211]);
    grid on; grid minor;
    ylabel(['Joint Error (m/s deg/s deg/s deg/s)']);
    xlabel(['Time (sec)']);
    legend({'Joint1','Joint2','Joint3','Joint4'},'Location','northeast');

    %saveas(f_d_error_deg,fullfile(fname,['error_control_state_feedback_deg_dq_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


    % Plot Control Signal

    f_control = figure();

    subplot(2,2,1);
    i = 1;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Hip control signal (N)']);
    xlabel(['Time (sec)']);

    subplot(2,2,2);
    i = 2;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Thigh control signal (N.m)']);
    xlabel(['Time (sec)']);

    subplot(2,2,3);
    i = 3;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Knee control signal (N.m)']);
    xlabel(['Time (sec)']);

    subplot(2,2,4);
    i = 4;
    plot(u.Time, u.Data(:,i));
    grid on; grid minor;
    ylabel(['Ankle control signal (N.m)']);
    xlabel(['Time (sec)']);
end
