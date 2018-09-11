function [dAd] = dAd(p_ij,R_ij,dp_ij,dR_ij)
%dAd(p_ij,R_ij,dp_ij,dR_ij) is the time derivative of the adjoint Ad_gij 6x6 matrix

dAd(1:3,1:3) = dR_ij;
dAd(4:6,4:6) = dR_ij;
dAd(1:3,4:6) = hat(dp_ij)*R_ij + hat(p_ij)*dR_ij;

end