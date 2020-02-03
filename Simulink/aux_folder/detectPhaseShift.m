function phase = detectPhaseShift(x, y)
% detectPhaseShift Detects the phase shift from x to y
%
% x = a1 * sin(t + phi1) + b1
% y = a2 * sin(t + phi2) + b2
%
% The algorithm returns phi2 - phi1 in degrees

    xM = max(x);
    xm = min(x);
    %
    yM = max(y);
    ym = min(y);
    %
    b1 = (xM + xm) / 2;
    b2 = (yM + ym) / 2;
    %
    a1 = xM - b1;
    a2 = yM - b2;
    %
    x = (x - b1) / a1;
    y = (y - b2) / a2;
    
    distance = x.^2 + y.^2;
    phase = atan( sqrt( max(distance) / min(distance) ) );
end