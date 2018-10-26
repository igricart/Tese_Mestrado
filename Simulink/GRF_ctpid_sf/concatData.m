%%Concatenation and signal processing function
% Smooths the the first and last window size samples from data
% and concatenate until t_final


function data_mod = concatData(data,t,window,t_final)

dif = (data(end,2:end)-data(1,2:end))/2;
delta = dif/window;
data_mod = data;

time_step = (size(t,2)-1)/t(end);

step_vec = repmat((1:window)', 1, size(delta,2));
step = step_vec.*delta;

data_mod(1:window,:) = data(1: window,:) + [zeros(size(step,1),1) step(window:-1:1,:)];
data_mod(end-window+1:end,:) = data(end-window+1:end,:) - [zeros(size(step,1),1) step(1:1:window,:)];

data_mod = repmat(data_mod(1:end-1,:),t_final/t(end),1);
data_mod = [data_mod; data_mod(1,:)];

data_mod(:,1) = (0:1/time_step:t_final)';

% for i=1:n
%     data_mod(i,2:size_col) = data_mod(i,2:size_col) + i*delta;
%     data_mod(size_row-i,2:size_col) = data_mod(size_row-i,2:size_col) - i*delta;
% end
% 
% data_steps = [data(1:size_row-1,:);
%             data(size_row,1)  (data(size_row,2:size_col)+data(1,2:size_col))/2;
%             data(2:size_row,1)+data(size_row,1) data(2:end,2:size_col)];
end
