function [Ad_tilde] = Ad_tilde(X)
%Ad_tilde(X) calculates the Ad tilde matrix of a X vector

Ad_tilde(1:3,4:6) = -hat(X(1:3));
Ad_tilde(4:6,1:3) = -hat(X(1:3));
Ad_tilde(4:6,4:6) = -hat(X(4:6));

end