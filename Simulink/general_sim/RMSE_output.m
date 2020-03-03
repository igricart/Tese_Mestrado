%% Run script
% Run simulation with sim_time default = 4
run Init_sim.m;
run Init_HGO.m;
sim_time = 4;

param_error = 1;
Mass_ctrl = Mass_ctrl.*param_error;
Inertia_ctrl = Inertia_ctrl.*param_error;
R_ctrl = R_ctrl.*param_error;
L_ctrl = L_ctrl.*param_error;
h_ctrl = h_ctrl.*param_error;

k_noise = 0;    

% Run simulation
sim('simulation_state2017a',sim_time);

param_error = 1.0406;
Mass_ctrl = Mass_ctrl.*param_error;
Inertia_ctrl = Inertia_ctrl.*param_error;
R_ctrl = R_ctrl.*param_error;
L_ctrl = L_ctrl.*param_error;
h_ctrl = h_ctrl.*param_error;

k_noise = 2e-7;
sim('simulation_state2017a',sim_time);
save(['state_feedback_' num2str(k_noise) '.mat']);

k_noise = 1e-6;
sim('simulation_state2017a',sim_time);
save(['state_feedback_' num2str(k_noise) '.mat']);

% Noise k_noise = [Noise1 Noise2 ... NoiseN]
k_noise = [2e-07 1e-06];

for j=1:size(k_noise,2)
    
    % HGO 4e-4
    Mu_hgo = 4e-4;
    Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
    sim('simulation_hgo2017a.slx',sim_time);
    save(['output_feedback_' num2str(sprintf('%.0d',Mu_hgo)) '_' num2str(k_noise(j)) '.mat']);

    % HGO 10e-4
    Mu_hgo = 10e-4;
    Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
    sim('simulation_hgo2017a.slx',sim_time);
    save(['output_feedback_' num2str(sprintf('%.0d',Mu_hgo)) '_' num2str(k_noise(j)) '.mat']);

    % HGO 19e-4
    Mu_hgo = 19e-4;
    Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
    sim('simulation_hgo2017a.slx',sim_time);
    save(['output_feedback_' num2str(sprintf('%.0d',Mu_hgo)) '_' num2str(k_noise(j)) '.mat']);
    
    % HGO Var
    Hmu = [eye(4)/Mu_hgo zeros(4); zeros(4) eye(4)/Mu_hgo^2];
    sim('simulation_hgo_var2017a.slx',sim_time);
    save(['output_feedback_var_' num2str(k_noise(j)) '.mat']);
end


%% Compute RMSE and RMSE_norm
disp('Results for 1e-6');
converter = 180/pi;

% Compute RMSE_track from Clean simulation
load('state_feedback.mat');
[RMSE_track_clean,RMSE_track_clean_norm] = rmse(q.Data,q_ref.Data);
RMSE_track_clean = [RMSE_track_clean(1) RMSE_track_clean(2:4)*converter];

% Compute RMSE_track from State Feedback simulation with noise and model uncertainties
load('state_feedback_1e-06.mat');
[RMSE_track_pert,RMSE_track_pert_norm] = rmse(q.Data,q_ref.Data);
RMSE_track_pert = [RMSE_track_pert(1) RMSE_track_pert(2:4)*converter];

% Compute RMSE_track from HGO with fixed gain simulation
for i = 1:3
    if i == 1
        load('output_feedback_4e-04_1e-06.mat')
    elseif i == 2
        load('output_feedback_1e-03_1e-06.mat')
    elseif i == 3
        load('output_feedback_2e-03_1e-06.mat')
    end
    % Track
    [RMSE,RMSE_norm] = rmse(q.Data,q_ref.Data);
    RMSE_track_fix(i,1:4) = RMSE;
    RMSE_track_fix_norm(i,1:4) = RMSE_norm;
    RMSE_track_fix(i,1:4) = [RMSE_track_fix(i,1) RMSE_track_fix(i,2:4)*converter];
    % Position estimation
    [RMSE,RMSE_norm] = rmse(q_hat.Data,q.Data);
    RMSE_obs_fix(i,1:4) = RMSE;
    RMSE_obs_fix_norm(i,1:4) = RMSE_norm;
    RMSE_obs_fix(i,1:4) = [RMSE_obs_fix(i,1) RMSE_obs_fix(i,2:4)*converter];
    % Velocity estimation
    [RMSE,RMSE_norm] = rmse(dq_hat.Data,dq.Data);
    RMSE_obs_dq_fix(i,1:4) = RMSE;
    RMSE_obs_dq_fix_norm(i,1:4) = RMSE_norm;
    RMSE_obs_dq_fix(i,1:4) = [RMSE_obs_dq_fix(i,1) RMSE_obs_dq_fix(i,2:4)*converter];
end
% Compute RMSE_track from HGO with variable gain simulation
load('output_feedback_var_1e-06.mat');
[RMSE_track_var,RMSE_track_var_norm] = rmse(q.Data,q_ref.Data);
[RMSE_obs_var,RMSE_track_obs_norm] = rmse(q_hat.Data,q.Data);
[RMSE_obs_dq_var,RMSE_track_obs_dq_norm] = rmse(dq_hat.Data,dq.Data);
RMSE_track_var = [RMSE_track_var(1) RMSE_track_var(2:4)*converter];
RMSE_obs_var = [RMSE_obs_var(1) RMSE_obs_var(2:4)*converter];
RMSE_obs_dq_var = [RMSE_obs_dq_var(1) RMSE_obs_dq_var(2:4)*converter];

% Show results
disp('Tracking clean');
disp(RMSE_track_clean);
disp('Tracking state feedback with noise and uncertainties');
disp(RMSE_track_pert);
disp('Tracking output feedback fixed (4e-4, 10e-4, 19e-4)');
disp(RMSE_track_fix);
disp('Tracking output variable');
disp(RMSE_track_var);
disp('Estimation fixed (4e-4, 10e-4, 19e-4) q');
disp(RMSE_obs_fix);
disp('Estimation variable q');
disp(RMSE_obs_var);
disp('Estimation fixed (4e-4, 10e-4, 19e-4) dq');
disp(RMSE_obs_dq_fix);
disp('Estimation variable dq');
disp(RMSE_obs_dq_var);

% Results for 2e-7
disp('Results for 2e-7');
converter = 180/pi;

% Compute RMSE_track from Clean simulation
load('state_feedback.mat');
[RMSE_track_clean,RMSE_track_clean_norm] = rmse(q.Data,q_ref.Data);
RMSE_track_clean = [RMSE_track_clean(1) RMSE_track_clean(2:4)*converter];

% Compute RMSE_track from State Feedback simulation with noise and model uncertainties
load('state_feedback_2e-07.mat');
[RMSE_track_pert,RMSE_track_pert_norm] = rmse(q.Data,q_ref.Data);
RMSE_track_pert = [RMSE_track_pert(1) RMSE_track_pert(2:4)*converter];

% Compute RMSE_track from HGO with fixed gain simulation
for i = 1:3
    if i == 1
        load('output_feedback_4e-04_2e-07.mat')
    elseif i == 2
        load('output_feedback_1e-03_2e-07.mat')
    elseif i == 3
        load('output_feedback_2e-03_2e-07.mat')
    end
    % Track
    [RMSE,RMSE_norm] = rmse(q.Data,q_ref.Data);
    RMSE_track_fix(i,1:4) = RMSE;
    RMSE_track_fix_norm(i,1:4) = RMSE_norm;
    RMSE_track_fix(i,1:4) = [RMSE_track_fix(i,1) RMSE_track_fix(i,2:4)*converter];
    % Position estimation
    [RMSE,RMSE_norm] = rmse(q_hat.Data,q.Data);
    RMSE_obs_fix(i,1:4) = RMSE;
    RMSE_obs_fix_norm(i,1:4) = RMSE_norm;
    RMSE_obs_fix(i,1:4) = [RMSE_obs_fix(i,1) RMSE_obs_fix(i,2:4)*converter];
    % Velocity estimation
    [RMSE,RMSE_norm] = rmse(dq_hat.Data,dq.Data);
    RMSE_obs_dq_fix(i,1:4) = RMSE;
    RMSE_obs_dq_fix_norm(i,1:4) = RMSE_norm;
    RMSE_obs_dq_fix(i,1:4) = [RMSE_obs_dq_fix(i,1) RMSE_obs_dq_fix(i,2:4)*converter];
end
% Compute RMSE_track from HGO with variable gain simulation
load('output_feedback_var_2e-07.mat');
[RMSE_track_var,RMSE_track_var_norm] = rmse(q.Data,q_ref.Data);
[RMSE_obs_var,RMSE_track_obs_norm] = rmse(q_hat.Data,q.Data);
[RMSE_obs_dq_var,RMSE_track_obs_dq_norm] = rmse(dq_hat.Data,dq.Data);
RMSE_track_var = [RMSE_track_var(1) RMSE_track_var(2:4)*converter];
RMSE_obs_var = [RMSE_obs_var(1) RMSE_obs_var(2:4)*converter];
RMSE_obs_dq_var = [RMSE_obs_dq_var(1) RMSE_obs_dq_var(2:4)*converter];

% Show results
disp('Tracking clean');
disp(RMSE_track_clean);
disp('Tracking state feedback with noise and uncertainties');
disp(RMSE_track_pert);
disp('Tracking output feedback fixed (4e-4, 10e-4, 19e-4)');
disp(RMSE_track_fix);
disp('Tracking output variable');
disp(RMSE_track_var);
disp('Estimation fixed (4e-4, 10e-4, 19e-4) q');
disp(RMSE_obs_fix);
disp('Estimation variable q');
disp(RMSE_obs_var);
disp('Estimation fixed (4e-4, 10e-4, 19e-4) dq');
disp(RMSE_obs_dq_fix);
disp('Estimation variable dq');
disp(RMSE_obs_dq_var);
