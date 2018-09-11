function [delRz] = delRz(q)
%dRz(q) is the partial derivative of the rotation matrix Rz(psi) over psi

sq = sin(q);
cq = cos(q);

delRz = [-sq -cq 0;...
         cq -sq 0;...
         0 0 0 ];

end