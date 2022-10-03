function [theta1, theta2] = sp_2E(p0, p1, p2, k1, k2)
% subproblem.sp_2E  Subproblem 2 Extended: Two Offset Cones
%   [theta1, theta2] = subproblem.sp_2E(p0, p1, p2, k1, k2) finds
%   theta1, theta2 such that
%       p0 + rot(k1, theta1)*p1 = rot(k2, theta2)*p2
%
%   This subproblem solution assumes there is 1 solution
%   If there are two solutions (only if the rotation axes intersect),
%   then rewrite the problem and use subproblem.sp_2()
%
%   The problem is ill-posed if (p1, k1), (p2, k2), or (k1, k2) are
%   parallel

KxP1 = cross(k1,p1);
KxP2 = cross(k2,p2);

A_1 = [KxP1 -cross(k1,KxP1)];
A_2 = [KxP2 -cross(k2,KxP2)];

A = [A_1 -A_2];

p = -k1*dot(k1,p1) + k2*dot(k2,p2) - p0;

radius_1_sq = dot(KxP1, KxP1);
radius_2_sq = dot(KxP2, KxP2);

alpha = radius_1_sq/(radius_1_sq+radius_2_sq);
beta = radius_2_sq/(radius_1_sq+radius_2_sq);
M_inv = eye(3)+k1*k1'*(alpha/(1-alpha));
AAT_inv = 1/(radius_1_sq+radius_2_sq)*(M_inv + M_inv*k2*k2'*M_inv*beta/(1-k2'*M_inv*k2*beta));
x_ls = A'*AAT_inv*p;

n_sym = cross(k1,k2);
pinv_A1 = A_1'/radius_1_sq;
pinv_A2 = A_2'/radius_2_sq;
A_perp_tilde = [pinv_A1; pinv_A2]*n_sym;

num = (norm(x_ls(3:4))^2-1)*norm(A_perp_tilde(1:2))^2 - (norm(x_ls(1:2))^2-1)*norm(A_perp_tilde(3:4))^2;
den = 2*(x_ls(1:2)'*A_perp_tilde(1:2)*norm(A_perp_tilde(3:4))^2 - x_ls(3:4)'*A_perp_tilde(3:4)*norm(A_perp_tilde(1:2))^2);

xi = num/den;

sc = x_ls + xi*A_perp_tilde;

theta1 = atan2(sc(1), sc(2));
theta2 = atan2(sc(3), sc(4));
end