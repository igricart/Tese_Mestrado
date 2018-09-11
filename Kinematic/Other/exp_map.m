function g01 = exp_map(p,w,q)

hat_w = hat(w);
I3 = eye(3);

R01 = I3 + sin(q)*hat_w + (1-cos(q))*hat_w^2;
p01 = (-sin(q)*I3 + (cos(q)-1)*hat_w)*(hat_w*p);

g01(1:3,1:3) = R01;
g01(1:3,4) = p01;
g01(4,:) = [0 0 0 1];