function [delRy] = delRy(q)
%delRy(q) is the partial derivative of the rotation matrix over Ry(theta)
%over theta

sq = sin(q);
cq = cos(q);

delRy = [-sq 0 cq;...
         0 0 0;...
         -cq 0 -sq];

end