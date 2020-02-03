function [power_data] = calcPower(data)
power_data = zeros(size(data,2),1);
for i=1:size(data,1)
    power_data = power_data + data(i,1:end).^2';
end

power_data = power_data/size(data,1);