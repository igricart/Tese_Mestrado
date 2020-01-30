function plotGaitData(data)
    n_joints = size(data,2) - 1;
    %
    for k = 1:n_joints
        figure();
        plot(data(:,1), data(:,k), 'b-');
        grid on;
    end
end

