function [theta, is_LS] = sp_3(p1, p2, k, d)
% subproblem.sp_3  Subproblem 3: Cone and Sphere
%   theta = subproblem.sp_3(p1, p2, k, d) finds theta such that
%       || rot(k, theta)*p1 - p2 || = d
%   If there's no solution, minimize the least-squares residual
%       | || rot(k, theta)*p1 - p2 || - d |
%
%   If the problem is well-posed, there may be 1 or 2 exact solutions, or 1
%   least-squares solution
%   theta1 and theta2 are column vectors of the solutions
%
%   [theta, is_LS] = subproblem.sp_3(p1, p2, k, d) also produces
%   a flag is_LS, which is true if theta is a least-squares solution
%
%   The problem is ill-posed if (p1, k) or (p2, k) are parallel

KxP = cross(k,p1);

A_1 = [KxP -cross(k,KxP)];
A = -2*p2'*A_1;
norm_A_sq = dot(A,A);
norm_A = sqrt(norm_A_sq);

b = d^2 - norm(p2-k*k'*p1)^2 - norm(KxP)^2;

x_ls = A_1'*(-2*p2*b/norm_A_sq);

if dot(x_ls,x_ls)>1
    theta = atan2(x_ls(1), x_ls(2));
    if nargout > 1
        is_LS = true;
    end
    return
end

xi = sqrt(1-b^2/norm_A_sq);

A_perp_tilde = [A(2); -A(1)];
A_perp = A_perp_tilde/ norm_A;

sc_1 = x_ls + xi*A_perp;
sc_2 = x_ls - xi*A_perp;

theta = [atan2(sc_1(1), sc_1(2)) atan2(sc_2(1), sc_2(2))];
if nargout > 1
    is_LS = false;
end
end