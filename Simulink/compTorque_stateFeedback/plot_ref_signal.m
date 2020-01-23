%% Plot Reference signal

% ---- q

f_q_ref_signal = figure();

subplot(2,2,1);
i = 1;
plot(q_ref.Time, q_ref.Data(:,i));
grid on; grid minor;
ylabel(['Hip displacement (m)']);
xlabel(['Time (sec)']);

subplot(2,2,2);
i = 2;
plot(q_ref.Time, q_ref.Data(:,i));
grid on; grid minor;
ylabel(['Thigh angle (rad)']);
xlabel(['Time (sec)']);

subplot(2,2,3);
i = 3;
plot(q_ref.Time, q_ref.Data(:,i));
grid on; grid minor;
ylabel(['Knee angle (rad)']);
xlabel(['Time (sec)']);

subplot(2,2,4);
i = 4;
plot(q_ref.Time, q_ref.Data(:,i));
grid on; grid minor;
ylabel(['Ankle angle (rad)']);
xlabel(['Time (sec)']);

saveas(f_q_ref_signal,'ref_signal','epsc');

% ---- dq
f_dq_ref_signal = figure();

subplot(2,2,1);
i = 1;
plot(dq_ref.Time, dq_ref.Data(:,i));
grid on; grid minor;
ylabel(['Hip vel (m/s)']);
xlabel(['Time (sec)']);

subplot(2,2,2);
i = 2;
plot(dq_ref.Time, dq_ref.Data(:,i));
grid on; grid minor;
ylabel(['Thigh angular vel(rad/s)']);
xlabel(['Time (sec)']);

subplot(2,2,3);
i = 3;
plot(dq_ref.Time, dq_ref.Data(:,i));
grid on; grid minor;
ylabel(['Knee angular vel (rad/s)']);
xlabel(['Time (sec)']);

subplot(2,2,4);
i = 4;
plot(dq_ref.Time, dq_ref.Data(:,i));
grid on; grid minor;
ylabel(['Ankle angular vel (rad/s)']);
xlabel(['Time (sec)']);

saveas(f_dq_ref_signal,'vel_ref_signal','epsc');


% ---- ddq

f_ddq_ref_signal = figure();

subplot(2,2,1);
i = 1;
plot(ddq_ref.Time, ddq_ref.Data(:,i));
grid on; grid minor;
ylabel(['Hip accel (m/s^2)']);
xlabel(['Time (sec)']);

subplot(2,2,2);
i = 2;
plot(ddq_ref.Time, ddq_ref.Data(:,i));
grid on; grid minor;
ylabel(['Thigh accel(rad/s^2)']);
xlabel(['Time (sec)']);

subplot(2,2,3);
i = 3;
plot(ddq_ref.Time, ddq_ref.Data(:,i));
grid on; grid minor;
ylabel(['Knee accel (rad/s^2)']);
xlabel(['Time (sec)']);

subplot(2,2,4);
i = 4;
plot(ddq_ref.Time, ddq_ref.Data(:,i));
grid on; grid minor;
ylabel(['Ankle  accel (rad/s^2)']);
xlabel(['Time (sec)']);

saveas(f_ddq_ref_signal,'accel_ref_signal','epsc');