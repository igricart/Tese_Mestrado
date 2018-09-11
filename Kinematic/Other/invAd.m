function [ inv ] = invAd(g_0b)
%invAd(g_0b) is the inverse adjoint Ad_gb0 6x6 matrix of a 4x4 homogeneous transformation g0b 

R_0b = g_0b(1:3,1:3);
p_0b = g_0b(1:3,4);

% inv = zeros(6); %needs correction here to deal with symbolic or double vars. we should evaluate if the input is sym or double to return the same type in output
% inv(1:3,1:3)=R_0b';
% inv(4:6,4:6)=R_0b';
% inv(1:3,4:6)=-R_0b'*hat(p_0b);

inv = [ R_0b.' -R_0b.'*hat(p_0b) ; zeros(3) R_0b.'];

end