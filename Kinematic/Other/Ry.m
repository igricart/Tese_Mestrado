function [Ry] = Ry(q)
%Ry(q) is the rotation matrix over y axis of q radians
Ry = [cos(q) 0 sin(q);0 1 0;-sin(q) 0 cos(q)];
end