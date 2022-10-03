function [theta, is_LS] = sp_4(h, p, k, d)
% subproblem.sp_4  Subproblem 4: Cone and Plane
%   theta = = subproblem.sp_4(h, p, k, d) finds theta such that
%       h'*rot(k,theta)*p = d
%   If there's no solution, minimize the least-squares residual
%       | h'*rot(k,theta)*p - d |
%
%   If the problem is well-posed, there may be 1 or 2 exact solutions, or 1
%   least-squares solution
%   theta1 and theta2 are column vectors of the solutions
%
%   [theta, is_LS] = subproblem.sp_4(h, p, k, d) also produces
%   a flag is_LS, which is true if theta is a least-squares solution
%
%   The problem is ill-posed if (p, k) or (h, k) are parallel

A_11 = cross(k,p);
A_1 = [A_11 -cross(k,A_11)];
A = h'*A_1;

b = d - h'*k*(k'*p);

norm_A_2 = dot(A,A);

x_ls = A_1'*(h*b);

if norm(norm_A_2) > b^2
    xi = sqrt(norm_A_2-b^2);
    A_perp_tilde = [A(2); -A(1)];

    
    sc_1 = x_ls + xi*A_perp_tilde;
    sc_2 = x_ls - xi*A_perp_tilde;

    theta = [atan2(sc_1(1), sc_1(2)) atan2(sc_2(1), sc_2(2))];
    if nargout > 1
        is_LS = false;
    end
else
    theta = atan2(x_ls(1), x_ls(2));
    if nargout > 1
        is_LS = true;
    end
end

end