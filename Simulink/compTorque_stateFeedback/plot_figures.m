%% Script to plot data from .mat
% Run script
run Init_4DOF_computedTorque_stateFeedback.m;
fname = '/home/ignacio/Documents/Msc/Tese_Mestrado/Simulink/compTorque_stateFeedback/figs';
%% Set simulation variables

is = 'correct';
control = 'PD';
mode = 'weak';
uncertainty = '10';
k_noise = 0.00001;

%% Initial state
if strcmp(is,'correct')
    % Correct
    qInit = [ 0.0216 0.5675 -0.13 -0.39 ]';
elseif strcmp(is,'zero')
    % Zero
    qInit = [0 0 0 0]';
end

%% Set different control gains
if strcmp(control,'PD')
    if strcmp(mode,'weak')
        wn = 2*pi*8;
        zeta = 0.9;
    elseif strcmp(mode,'medium')
        wn = 2*pi*8;
        zeta = 0.9;
    elseif strcmp(mode,'strong')
        wn = 2*pi*8;
        zeta = 0.9;
    end
    ki = 0;
    kp = wn^2;
    Kp = kp*eye(n); Kd = kd*eye(n); Ki = ki*eye(n);
elseif strcmp(control,'PID')
    if strcmp(mode,'weak')
        wn = 2*pi*8;
        zeta = 0.9;
        p = 2*wn;
    elseif strcmp(mode,'medium')
        wn = 2*pi*8;
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

%% Add uncertainties
if strcmp(uncertainty,'0')
    param_error = (1 + 0*(rand()-1/2));
elseif strcmp(uncertainty,'10')
    param_error = (1 + 0.1*(rand()-1/2));
end
%% Add noise
noise = num2str(k_noise);


%% Run simulation
sim('Leg_4DOF_computedTorque_stateFeedback',2)


%% Plot Joints

% Plot 1 - Hip Displacement and Vel
i = 1;
f_hip = figure();

subplot(1,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i));
grid on; grid minor;
ylabel(['Hip displacement (m)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(1,2,2);
plot(dq_ref.Time, dq_ref.Data(:,i),'k', dq.Time, dq.Data(:,i));
grid on; grid minor;
ylabel(['Hip vel (m/s)']);
xlabel(['Time (sec)']);
legend({'Desired','True'},'Location','southeast');

saveas(f_hip,fullfile(fname,['hip_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');

% Plot 2 - Thigh angle
% Thigh joint zoomed
i = 2;
f_thigh = figure();

subplot(1,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i));
grid on; grid minor;
ylabel(['Thing angle (rad)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(1,2,2);
plot(dq_ref.Time, dq_ref.Data(:,i),'k', dq.Time, dq.Data(:,i));
grid on; grid minor;
ylabel(['Thigh angular vel (rad/s)']);
xlabel(['Time (sec)']);
legend({'Desired','True'},'Location','southeast');

saveas(f_thigh,fullfile(fname,['thigh_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


% Plot 3 - Knee angle
% Knee joint zoomed
i = 3;
f_knee = figure();

subplot(1,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i));
grid on; grid minor;
ylabel(['Knee angle (rad)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(1,2,2);
plot(dq_ref.Time, dq_ref.Data(:,i),'k', dq.Time, dq.Data(:,i));
grid on; grid minor;
ylabel(['Knee angular vel (rad/s)']);
xlabel(['Time (sec)']);
legend({'Desired','True'},'Location','southeast');

saveas(f_knee,fullfile(fname,['knee_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


% Plot 4 - Ankle angle
% Ankle joint zoomed
i = 4;
f_ankle = figure();

subplot(1,2,1);
plot(q_ref.Time, q_ref.Data(:,i),'k', q.Time, q.Data(:,i));
grid on; grid minor;
ylabel(['Ankle angle (rad)']);
xlabel(['Time (sec)']);

% Zoomed
subplot(1,2,2);
plot(dq_ref.Time, dq_ref.Data(:,i),'k', dq.Time, dq.Data(:,i));
grid on; grid minor;
ylabel(['Ankle angular vel (rad/s)']);
xlabel(['Time (sec)']);
legend({'Desired','True'},'Location','southeast');

saveas(f_ankle,fullfile(fname,['ankle_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


%% Plot Error

% Plot 1 - hip angle
i = 1;
f_hip_error = figure();

% q
subplot(1,2,1);
plot(q_error.Time, q_error.Data(:,i));
grid on; grid minor;
ylabel(['Hip displacement (m)']);
xlabel(['Time (sec)']);

% dq
subplot(1,2,2);
plot(dq_error.Time, dq_error.Data(:,i));
grid on; grid minor;
ylabel(['Hip vel (rad/s)']);
xlabel(['Time (sec)']);
legend({'Error'},'Location','southeast');

saveas(f_hip_error,fullfile(fname,['hip_error_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


% Plot 2 - Thigh angle
% Thigh joint zoomed
i = 2;
f_thigh_error = figure();

% q
subplot(1,2,1);
plot(q_error.Time, q_error.Data(:,i));
grid on; grid minor;
ylabel(['Thing angle (rad)']);
xlabel(['Time (sec)']);

% dq
subplot(1,2,2);
plot(dq_error.Time, dq_error.Data(:,i));
grid on; grid minor;
ylabel(['Thigh angular vel (rad/s)']);
xlabel(['Time (sec)']);
legend({'Error'},'Location','southeast');

saveas(f_thigh_error,fullfile(fname,['thigh_error_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');

% Plot 3 - Knee angle
i = 3;
f_knee_error = figure();

% q
subplot(1,2,1);
plot(q_error.Time, q_error.Data(:,i));
grid on; grid minor;
ylabel(['Thing angle (rad)']);
xlabel(['Time (sec)']);

% dq
subplot(1,2,2);
plot(dq_error.Time, dq_error.Data(:,i));
grid on; grid minor;
ylabel(['Knee angular vel (rad/s)']);
xlabel(['Time (sec)']);
legend({'Error'},'Location','southeast');

saveas(f_knee_error,fullfile(fname,['knee_error_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');

% Plot 4 - ankle angle
i = 4;
f_ankle_error = figure();

% q
subplot(1,2,1);
plot(q_error.Time, q_error.Data(:,i));
grid on; grid minor;
ylabel(['Ankle angle (rad)']);
xlabel(['Time (sec)']);

% dq
subplot(1,2,2);
plot(dq_error.Time, dq_error.Data(:,i));
grid on; grid minor;
ylabel(['Ankle angular vel (rad/s)']);
xlabel(['Time (sec)']);
legend({'Error'},'Location','southeast');

saveas(f_ankle_error,fullfile(fname,['ankle_error_control_state_feedback_' is '_' control '_' mode '_' uncertainty '_' noise]),'epsc');


%% Plot Control Signal

f_control = figure();

subplot(2,2,1);
i = 1;
plot(u.Time, u.Data(:,i));
grid on; grid minor;
ylabel(['Hip control signal (N.m)']);
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
