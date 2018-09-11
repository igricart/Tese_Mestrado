function [dinvAd] = dinvAd(p_ij,R_ij,dp_ij,dR_ij)
%dAd(p_ij,R_ij,dp_ij,dR_ij) is the time derivative of the adjoint Ad_gij 6x6 matrix

dinvAd(1:3,1:3) = dR_ij';
dinvAd(4:6,4:6) = dR_ij';
dinvAd(1:3,4:6) = -(dR_ij'*hat(p_ij) + R_ij'*hat(dp_ij));

end