function [Rx] = Rx(q)
%Rx(q) is the rotation matrix over x axis of q radians
Rx = [1 0 0;0 cos(q) -sin(q);0 sin(q) cos(q)];
end