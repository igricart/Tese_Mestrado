load('mat_files/Quadril_Sagital-1.dat')
load('mat_files/Joelho_Sagital-1.dat')
load('mat_files/Tornozelo_Sagital-1.dat')

% Requires uptade in .mat files
%load('mat_files/acc_state_feedback_clean.mat')

data = downsample(data_steps(1:10001,:),200);
t = linspace(1,100,51);

f_hip = figure();
plot(t, (data(:,3) - 2*0.0175)*180/pi, t, Quadril_Sagital_1(:,2:end))
grid on; grid minor;
xlabel('Gait %');
ylabel('Hip joint angle (deg)');
legend('Patient','Lower margin','Upper margin','Location','Southwest');
saveas(f_hip, 'figs/lepo_hip','epsc');

f_knee = figure();
plot(t, (data(:,4)+ 6*0.0175)*180/pi, t, -Joelho_Sagital_1(:,2:end))
grid on; grid minor;
xlabel('Gait %');
ylabel('Knee joint angle (deg)');
legend('Patient','Lower margin','Upper margin','Location','Southwest');
saveas(f_knee, 'figs/lepo_knee','epsc');

f_ankle = figure();
plot(t, (data(:,5) + 19.5*0.0175)*180/pi, t, Tornozelo_Sagital_1(:,2:end))
grid on; grid minor;
xlabel('Gait %');
ylabel('Ankle joint angle (deg)');
legend('Patient','Lower margin','Upper margin','Location','Southwest');
saveas(f_ankle, 'figs/lepo_ankle','epsc');

f_ref = figure();
plot(q_ref.Time, q_ref.Data, q_raw.Time, q_raw.Data);
grid on; grid minor;
xlabel('Gait %');
ylabel('Ankle joint angle (deg)');
legend('Patient','Lower margin','Upper margin','Location','Southwest');
saveas(f_ref, 'figs/filtered_ref_signal','epsc');