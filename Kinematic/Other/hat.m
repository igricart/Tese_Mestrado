function [Xhat] = hat(X)
%hat(X) is the skew symmetric operator for a 3x1 vector X
Xhat = [0 -X(3) X(2);X(3) 0 -X(1);-X(2) X(1) 0];
end

