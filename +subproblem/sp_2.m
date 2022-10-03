function [theta1, theta2, is_LS] = sp_2(p1, p2, k1, k2)
% subproblem.sp_2  Subproblem 2: Two Cones
%   [theta1, theta2] = subproblem.sp_2(p1, p2, k1, k2) finds theta1, theta2
%   such that
%       rot(k1, theta1)*p1 = rot(k2, theta2)*p2
%   If there's no solution, minimize the least-squares residual
%           || rot(k1, theta1)*p1 - rot(k2, theta2)*p2 ||
%
%   If the problem is well-posed, there may be 1 or 2 solutions
%   (These may be exact or least-squares solutions)
%   theta1 and theta2 are column vectors of the solutions
%
%   [theta1, theta2, is_LS] = subproblem.sp_2(p1, p2, k1, k2) also produces
%   a flag is_LS, which is true if (theta1, theta2) is a least-squares
%   solution
%
%   The problem is ill-posed if (p1, k1), (p2, k2), or (k1, k2) are
%   parallel
    
if nargout > 1
    is_LS = abs(norm(p1) - norm(p2)) > 1e-8;
end

% Rescale for least-squares case
p1 = p1/norm(p1);
p2 = p2/norm(p2);

KxP1 = cross(k1,p1);
KxP2 = cross(k2,p2);

A_1 = [KxP1 -cross(k1,KxP1)];
A_2 = [KxP2 -cross(k2,KxP2)];

%A = [A_1 -A_2];
%p = -k1*dot(k1,p1) + k2*dot(k2,p2);

radius_1_sq = dot(KxP1, KxP1);
radius_2_sq = dot(KxP2, KxP2);

k1_d_p1 = dot(k1,p1);
k2_d_p2 = dot(k2,p2);
k1_d_k2 = dot(k1,k2);
ls_frac = 1/(1-k1_d_k2^2);
alpha_1 = ls_frac * (k1_d_p1-k1_d_k2*k2_d_p2);
alpha_2 = ls_frac * (k2_d_p2-k1_d_k2*k1_d_p1);
x_ls_1 = alpha_2*A_1'*k2 / radius_1_sq;
x_ls_2 = alpha_1*A_2'*k1 / radius_2_sq;
x_ls = [x_ls_1; x_ls_2];

n_sym = cross(k1,k2);
pinv_A1 = A_1'/radius_1_sq;
pinv_A2 = A_2'/radius_2_sq;
A_perp_tilde = [pinv_A1; pinv_A2]*n_sym;

if norm(x_ls(1:2)) < 1
    xi = sqrt(1 - norm(x_ls(1:2))^2 ) / norm(A_perp_tilde(1:2));
    sc_1 = x_ls + xi*A_perp_tilde;
    sc_2 = x_ls - xi*A_perp_tilde;
    
    theta1 = [atan2(sc_1(1), sc_1(2)) atan2(sc_2(1), sc_2(2))];
    
    theta2 = [atan2(sc_1(3), sc_1(4)) atan2(sc_2(3), sc_2(4))];
else
    theta1 = atan2(x_ls(1), x_ls(2));
    theta2 = atan2(x_ls(3), x_ls(4));
    if nargout > 1
        is_LS = true;
    end
end