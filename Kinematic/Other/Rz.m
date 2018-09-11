function [Rz] = Rz(q)
%Rz(q) is the rotation matrix over y axis of q radians
Rz = [cos(q) -sin(q) 0;sin(q) cos(q) 0;0 0 1];
end