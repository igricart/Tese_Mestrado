function [ XData, YData ] = drawLine( p1, p2, varargin )
% Draw line using origin and angle.
if ( (abs(nargin)-3) == 1 )
    spatial_res = varargin{1};
else
    spatial_res = 0.1;
end
line = p2 - p1;
base_line = 0:spatial_res:1;
XData = p1(1)*ones(1,length(base_line)) + line(1)*base_line;
YData = p1(2)*ones(1,length(base_line)) + line(2)*base_line;

end