% Compute RMSE and RMSE_norm

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
RMSE_track_var = [RMSE_track_var(1) RMSE_track_var(2:4)*converter];
RMSE_obs_var = [RMSE_obs_var(1) RMSE_obs_var(2:4)*converter];