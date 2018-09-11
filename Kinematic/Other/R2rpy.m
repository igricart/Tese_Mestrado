function RPY = R2rpy(R)
%R2rpy(R) gets the roll pitch yaw angle values from a rotation matrix R 

%Using atan2:
yaw = atan2(R(2,1),R(1,1));
roll = atan2(R(3,2),R(3,3));
pitch = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));

% %Using atan instead of atan2 due to symbolic algebrisms of atan2 resulting in complex numbers:
% yaw=atan(R(2,1)/R(1,1));
% roll=atan(R(3,2)/R(3,3));
% pitch=-asin(R(3,1));

RPY = [roll;pitch;yaw];

end