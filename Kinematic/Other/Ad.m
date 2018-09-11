function Ad = Ad(g_0b)
%Ad(g_0b) is the adjoint Ad_g0b 6x6 matrix of a 4x4 homogeneous transformation g0b 

R_0b = g_0b(1:3,1:3);
p_0b = g_0b(1:3,4);

Ad = zeros(6,6);
Ad(1:3,1:3) = R_0b;
Ad(4:6,4:6) = R_0b;
Ad(1:3,4:6) = hat(p_0b)*R_0b;

end