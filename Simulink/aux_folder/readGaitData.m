%Reads the fname sheet and creates a step period of time_period
% n_joints is necessary to acquire the right joints data

function [data, t] = readGaitData(fname, n_joints, time_period, Ts)
    %%Open graphics from GaitData
    gaitData = xlsread(fname);
    %%Column 2 = Young | Column 3 = Adult
    
    t = 0:Ts:time_period;
    n_rows = size(gaitData, 1) / n_joints;
    if (round(n_rows) ~= n_rows)
        error('Wrong number of joints');
    end
    n_cols = n_joints;
    data_raw = zeros(n_rows, n_cols);
    
    for k = 1:n_joints
        indices = (1:n_rows) + n_rows * (k-1);
        data_raw(:,k) = (2*pi/360)*gaitData(indices, 3);
    end
    
    data_raw = [linspace(0,100*time_period,n_rows)'/100 data_raw];
    
    data_concat = [data_raw(:,1) data_raw(:,3) data_raw(:,5) data_raw(:,8) data_raw(:,9)];
    for i=1:4
    data_sp{i} = spaps(data_concat(:,1),data_concat(:,i+1),1e-10);
    data_ov{i} = fnval(data_sp{i},t)';
    end
    
    %convert cell do timesires struct
    %OBS.: Terceira medida é invertida
    data = [t' data_ov{:,1} data_ov{:,2} -data_ov{:,3} data_ov{:,4}];
    
end