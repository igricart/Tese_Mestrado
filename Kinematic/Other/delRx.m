function [delRx] = delRx(q)
%delRx(q) is the partial derivative of the rotation matrix Rx(phi) over phi

sq = sin(q);
cq = cos(q);

delRx = [0 0 0;...
         0 -sq -cq;...
         0 cq -sq];
end

