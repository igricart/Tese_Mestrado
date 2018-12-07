 function [ceq1,ceq2] = mycon( q, l2, l3, ah, aH, aT, z_h, z_t)   

 % Joint constraints
anal_z_h = q(1) + l2 * cos( q(2) ) + l3 * ( cos( q(2) + q(3) ) ) ...
+ aH * cos( q(2) + q(3) + q(4) + ( - acos( ah / aH ) ) );

anal_z_t = q(1) + l2 * cos( q(2) ) + l3 * ( cos( q(2) + q(3) ) ) ...
+ aT * cos( q(2) + q(3) + q(4) + ( acos( ah / aT ) ) );

% ceq1 = 0, ceq2 = 0
ceq1 = anal_z_h - z_h;
ceq2 = anal_z_t - z_t;         
 end