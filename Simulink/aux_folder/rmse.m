function [RMSE, RMSE_norm] = rmse(y,y_d)
(y - y_d);    % Errors
(y - y_d).^2;   % Squared Error
mean((y - y_d).^2);   % Mean Squared Error
RMSE = sqrt(mean((y - y_d).^2));  % Root Mean Squared Error
RMSE_norm = RMSE./max(y_d);