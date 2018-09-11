function [V_0b, dV_0b] = inertial_to_body(eta, deta, ddeta)
%INERTIAL_TO_BASE Summary of this function goes here
%   Detailed explanation goes here

eta2 = eta(4:end);
deta2 = deta(4:end);

invJb = invJ_rpy(eta2);
% dJb = dJ_rpy(eta2,deta2);
dinvJb = dinvJ_rpy(eta2,deta2);

V_0b = invJb*deta;
% dV_0b = invJb*(ddeta - dJb*V_0b);
dV_0b = invJb*ddeta + dinvJb*deta;

end